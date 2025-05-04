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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class PROSACRobustEasyGyroscopeCalibratorTest implements RobustEasyGyroscopeCalibratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_ANGLE_VARIATION_DEGREES = -2.0;
    private static final double MAX_ANGLE_VARIATION_DEGREES = 2.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 5;

    private static final double THRESHOLD = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double OUTLIER_ERROR_FACTOR = 1000.0;

    private static final int TIMES = 100;

    private int calibrateStart;
    private int calibrateEnd;
    private int calibrateNextIteration;
    private int calibrateProgressChange;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(new double[3], bg1, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(new Matrix(3, 1), bg1Matrix);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertNull(calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m1, mg,
                gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m2, mg,
                gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m3,
                gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m4,
                gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m6));
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m1, mg,
                gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m2, mg,
                gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m3,
                gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m4,
                gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m6, this));
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                mg, m4));
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, m1,
                gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, m2,
                gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m4, this));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias,
                m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, baArray, m6));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg, gg, baArray, ma,
                this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, baArray, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, m1,
                gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, m2,
                gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bias, mg,
                gg, baArray, m6, this));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY,calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m1, mg,
                gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m2, mg,
                gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m3,
                gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m4,
                gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, ba, m10));
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m1, mg,
                gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, m2, mg,
                gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m3,
                gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, m4,
                gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences, bg, mg,
                gg, ba, m10, this));
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bg, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6));
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bg, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, this));
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bias, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4));
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bias, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, this));
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m6));
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, baArray, ma,
                this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m6, this));
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m10));
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(sequences, false,
                false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m10, this));
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(new double[3], bg1, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(new Matrix(3, 1), bg1Matrix);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertNull(calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1]));
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m6));
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg,
                this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m6, this));
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m4));
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg,
                this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx,calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m4, this));
    }

    @Test
    void testConstructor23() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg, baArray,
                ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bias, mg, gg, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, baArray, m6));
    }

    @Test
    void testConstructor24() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg, baArray,
                ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bias, mg, gg, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, new double[1], mg, gg, baArray, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m1, gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, m2, gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bias, mg, gg, baArray, m6, this));
    }

    @Test
    void testConstructor25() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, ba, m10));
    }

    @Test
    void testConstructor26() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg, ba, ma,
                this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, bg, mg, gg, ba, m10, this));
    }

    @Test
    void testConstructor27() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bg, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(),
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m6));
    }

    @Test
    void testConstructor28() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
        final var gg = generateGg();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bg, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m6, this));
    }

    @Test
    void testConstructor29() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bias, mg, gg);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m4));
    }

    @Test
    void testConstructor30() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bias, mg, gg, this);

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
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(new Matrix(3, 1), ba1Matrix);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
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
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, new double[1], mg, gg,
                this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m4, this));
    }

    @Test
    void testConstructor31() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bias, mg, gg, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, gg, baArray, m6));
    }

    @Test
    void testConstructor32() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bias, mg, gg, baArray, ma,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, new double[1], mg, gg, baArray, ma,
                this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m1, gg, baArray, ma,
                this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, m2, gg, baArray, ma,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m3, baArray, ma,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, m4, baArray, ma,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg,
                new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, gg, baArray, m5,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bias, mg, gg, baArray, m6,
                this));
    }

    @Test
    void testConstructor33() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, ba, m10));
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();

        final var bg = generateBg();
        final var mg = generateGeneralMg();
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
        final var baArray = ba.getBuffer();

        final var baX = ba.getElementAtIndex(0);
        final var baY = ba.getElementAtIndex(1);
        final var baZ = ba.getElementAtIndex(2);

        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences, false,
                false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final var accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final var accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final var accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final var accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final var accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final var accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final var ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final var ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final var ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
        final var ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final var ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);

        final var angularSpeedX1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final var angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final var angularSpeedY1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final var angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final var angularSpeedZ1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final var angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final var initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(bgx, initialBiasTriad1.getValueX(), 0.0);
        assertEquals(bgy, initialBiasTriad1.getValueY(), 0.0);
        assertEquals(bgz, initialBiasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasTriad1.getUnit());
        final var initialBiasTriad2 = new AngularSpeedTriad();
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

        final var bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final var bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final var bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepResiduals());
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());

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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(new double[1],
                sequences, false, false, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg,
                m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROSACRobustEasyGyroscopeCalibrator(qualityScores,
                sequences, false, false, bg, mg, gg, ba, m10, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_THRESHOLD, calibrator.getThreshold(), 0.0);

        // set new value
        calibrator.setThreshold(1.0);

        // check
        assertEquals(1.0, calibrator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setThreshold(0.0));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());

        // set new value
        calibrator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepResiduals());

        // set new value
        calibrator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepResiduals());
    }

    @Test
    void testGetSetAccelerometerBiasX() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var bax1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        // set new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bax3 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(bax3);

        // check
        final var bax4 = calibrator.getAccelerometerBiasXAsAcceleration();
        final var bax5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax5);
        assertEquals(bax3, bax4);
        assertEquals(bax3, bax5);
    }

    @Test
    void testGetSetAccelerometerBiasYAsAcceleration() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var bay1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        // set new value
        final var ba = generateBa();
        final var bay = ba.getElementAtIndex(1);
        final var bay3 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(bay3);

        // check
        final var bay4 = calibrator.getAccelerometerBiasYAsAcceleration();
        final var bay5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay5);
        assertEquals(bay3, bay4);
        assertEquals(bay3, bay5);
    }

    @Test
    void testGetSetAccelerometerBiasZAsAcceleration() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var baz1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        // set new value
        final var ba = generateBa();
        final var baz = ba.getElementAtIndex(2);
        final var baz3 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(baz3);

        // check
        final var baz4 = calibrator.getAccelerometerBiasZAsAcceleration();
        final var baz5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz5);
        assertEquals(baz3, baz4);
        assertEquals(baz3, baz5);
    }

    @Test
    void testSetAccelerometerBias1() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var bax1 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay1 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz1 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(bax1, bay1, baz1);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBias() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var ba1 = calibrator.getAccelerometerBias();
        final var ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);

        assertArrayEquals(new double[3], ba1, 0.0);
        assertArrayEquals(ba1, ba2, 0.0);

        // set new value
        final var ba3 = generateBa().getBuffer();
        calibrator.setAccelerometerBias(ba3);

        // check
        final var ba4 = calibrator.getAccelerometerBias();
        final var ba5 = new double[3];
        calibrator.getAccelerometerBias(ba5);

        assertArrayEquals(ba3, ba4, 0.0);
        assertArrayEquals(ba3, ba5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(new double[1]));
    }

    @Test
    void testGetSetAccelerometerBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final var ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);

        // check
        assertEquals(new Matrix(3, 1), ba1);
        assertEquals(ba1, ba2);

        // set new value
        final var ba3 = generateBa();
        calibrator.setAccelerometerBias(ba3);

        final var ba4 = calibrator.getAccelerometerBiasAsMatrix();
        final var ba5 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba5);

        assertEquals(ba3, ba4);
        assertEquals(ba3, ba5);
    }

    @Test
    void testGetSetAccelerometerSx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var sxa = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(sxa);

        // check
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerSy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var sya = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(sya);

        // check
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerSz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(sza);

        // check
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMxy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);

        // set new value
        final var ma = generateMa();
        final var mxya = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(mxya);

        // check
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMxz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var mxza = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(mxza);

        // check
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMyx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var myxa = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(myxa);

        // check
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMyz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);

        // set new value
        final var ma = generateMa();
        final var myza = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(myza);

        // check
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMzx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);

        // set new value
        final var ma = generateMa();
        final var mzxa = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(mzxa);

        // check
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMzy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new value
        final var ma = generateMa();
        final var mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(mzya);

        // check
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testSetAccelerometerScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new values
        final var ma = generateMa();
        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(sxa, sya, sza);

        // check
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    void testSetAccelerometerCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final var ma = generateMa();
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerCrossCouplingErrors(mxya, mxza, myxa, myza, mzxa, mzya);

        // check
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testSetAccelerometerScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var sxa = ma.getElementAt(0, 0);
        final var sya = ma.getElementAt(1, 1);
        final var sza = ma.getElementAt(2, 2);
        final var mxya = ma.getElementAt(0, 1);
        final var mxza = ma.getElementAt(0, 2);
        final var myxa = ma.getElementAt(1, 0);
        final var myza = ma.getElementAt(1, 2);
        final var mzxa = ma.getElementAt(2, 0);
        final var mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(sxa, sya, sza, mxya, mxza, myxa, myza, mzxa,
                mzya);

        // check
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    void testGetSetAccelerometerMa() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        assertEquals(new Matrix(3, 3), calibrator.getAccelerometerMa());
        final var ma1 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma1);
        assertEquals(new Matrix(3, 3), ma1);

        // set new value
        final var ma2 = generateMa();
        calibrator.setAccelerometerMa(ma2);

        // check
        assertEquals(ma2, calibrator.getAccelerometerMa());
        final var ma3 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma3);
        assertEquals(ma2, ma3);

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
    void testGetSetInitialBiasX() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(bgx);

        // check
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    void testGetSetInitialBiasY() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bgy = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(bgy);

        // check
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    void testGetSetInitialBiasZ() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final var bg = generateBg();
        final var bgz = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(bgz);

        // check
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedX() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final var bgx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);

        // set new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgx3 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasX(bgx3);

        // check
        final var bgx4 = calibrator.getInitialBiasAngularSpeedX();
        final var bgx5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx5);

        assertEquals(bgx3, bgx4);
        assertEquals(bgx3, bgx5);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedY() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final var bgy2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);

        // set new value
        final var bg = generateBg();
        final var bgy = bg.getElementAtIndex(1);
        final var bgy3 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasY(bgy3);

        // check
        final var bgy4 = calibrator.getInitialBiasAngularSpeedY();
        final var bgy5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy5);

        assertEquals(bgy3, bgy4);
        assertEquals(bgy3, bgy5);
    }

    @Test
    void testGetSetInitialBiasAngularSpeedZ() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        final var bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        final var bgz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);

        // set new value
        final var bg = generateBg();
        final var bgz = bg.getElementAtIndex(2);
        final var bgz3 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasZ(bgz3);

        // check
        final var bgz4 = calibrator.getInitialBiasAngularSpeedZ();
        final var bgz5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz5);

        assertEquals(bgz3, bgz4);
        assertEquals(bgz3, bgz5);
    }

    @Test
    void testSetInitialBias1() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        calibrator.setInitialBias(bgx, bgy, bgz);

        // check
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testSetInitialBias2() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var bgx1 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy1 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz1 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(bgx1, bgy1, bgz1);

        // check
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    void testGetSetInitialBiasAsTriad() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final var mg = generateGeneralMg();
        final var mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var mg = generateGeneralMg();
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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mg = generateGeneralMg();
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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

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
        final var mg = generateGeneralMg();
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
    void testGetSetInitialBias() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        final var bg1 = calibrator.getInitialBias();
        final var bg2 = new double[3];
        calibrator.getInitialBias(bg2);

        assertArrayEquals(new double[3], bg1, 0.0);
        assertArrayEquals(bg1, bg2, 0.0);

        // set new value
        final var bg3 = generateBg().getBuffer();
        calibrator.setInitialBias(bg3);

        // check
        final var bg4 = calibrator.getInitialBias();
        final var bg5 = new double[3];
        calibrator.getInitialBias(bg5);

        assertArrayEquals(bg3, bg4, 0.0);
        assertArrayEquals(bg3, bg5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialBias(new double[1]));
    }

    @Test
    void testGetSetInitialBiasAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial values
        final var bg1 = calibrator.getInitialBiasAsMatrix();
        final var bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);

        assertEquals(new Matrix(3, 1), bg1);
        assertEquals(bg1, bg2);

        // set new value
        final var bg3 = generateBg();
        calibrator.setInitialBias(bg3);

        // check
        final var bg4 = calibrator.getInitialBiasAsMatrix();
        final var bg5 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg5);

        assertEquals(bg3, bg4);
        assertEquals(bg3, bg5);

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        final var mg1 = calibrator.getInitialMg();
        final var mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);

        assertEquals(new Matrix(3, 3), mg1);
        assertEquals(mg1, mg2);

        // set new value
        final var mg3 = generateGeneralMg();
        calibrator.setInitialMg(mg3);

        // check
        final var mg4 = calibrator.getInitialMg();
        final var mg5 = new Matrix(3, 3);
        calibrator.getInitialMg(mg5);

        assertEquals(mg3, mg4);
        assertEquals(mg3, mg5);

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
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        final var gg1 = calibrator.getInitialGg();
        final var gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);

        assertEquals(new Matrix(3, 3), gg1);
        assertEquals(gg1, gg2);

        // set new value
        final var gg3 = generateGg();
        calibrator.setInitialGg(gg3);

        // check
        final var gg4 = calibrator.getInitialGg();
        final var gg5 = new Matrix(3, 3);
        calibrator.getInitialGg(gg5);

        assertEquals(gg3, gg4);
        assertEquals(gg3, gg5);

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
    void testGetSetSequences() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getSequences());

        // set new value
        final var sequences = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        calibrator.setSequences(sequences);

        // check
        assertSame(sequences, calibrator.getSequences());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    void testIsSetGDependentCrossBiasesEstimated() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetMinimumRequiredMeasurementsOrSequences() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check initial value
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
    void testIsReady() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        assertFalse(calibrator.isReady());
        assertEquals(19, calibrator.getMinimumRequiredMeasurementsOrSequences());

        // set empty sequences
        final var sequences1 = Collections.<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>emptyList();
        calibrator.setSequences(sequences1);

        // check
        assertFalse(calibrator.isReady());

        // set enough sequences
        final var sequences2 = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
        for (var i = 0; i < calibrator.getMinimumRequiredMeasurementsOrSequences(); i++) {
            sequences2.add(new BodyKinematicsSequence<>());
        }
        calibrator.setSequences(sequences2);

        // check
        assertFalse(calibrator.isReady());

        // add quality scores with invalid size
        var qualityScores = new double[sequences2.size() + 1];
        calibrator.setQualityScores(qualityScores);

        assertFalse(calibrator.isReady());

        // add quality scores with valid size
        qualityScores = new double[sequences2.size()];
        calibrator.setQualityScores(qualityScores);

        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, calibrator.getProgressDelta(), 
                0.0);

        // set new value
        calibrator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, calibrator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);

        // set new value
        calibrator.setConfidence(0.8);

        // check
        assertEquals(0.8, calibrator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROSACRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, calibrator.getMaxIterations());

        // set new value
        calibrator.setMaxIterations(1);

        assertEquals(1, calibrator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final var qualityScores = new double[EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, calibrator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setQualityScores(new double[9]));
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var calibrator = new PROSACRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES, 
                calibrator.getPreliminarySubsetSize());

        // set new value
        calibrator.setPreliminarySubsetSize(20);

        // check
        assertEquals(20, calibrator.getPreliminarySubsetSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(6));
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException, 
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = new Matrix(3, 3);
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
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

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, errorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, errorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    true, false, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = new Matrix(3, 3);
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
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

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, errorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, errorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    false, false, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = generateGg();
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
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

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, errorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, errorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    true, true, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

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
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = generateGg();
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
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

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, errorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                                trueKinematics, errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, errorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    false, true, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

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
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
            final var gg = new Matrix(3, 3);
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final var noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, noErrorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, noErrorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    true, false, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    void testCalibrateGeneralAndGDependentCrossBiasesDisabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
            final var gg = new Matrix(3, 3);
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final var errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final var noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, noErrorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, noErrorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    false, false, bg, mg, gg, ba, ma, this);
            final var subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateCommonAxisMg();
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
            final var noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, noErrorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, noErrorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    true, true, bg, mg, gg, ba, ma, this);
            final var subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            if (calibrator.getEstimatedCovariance() != null) {
                checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            }
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralAndGDependentCrossBiasesEnabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMa();
            final var mg = generateGeneralMg();
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
            final var noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final var errorRandomizer = new GaussianRandomizer(0.0, angularRateStandardDeviation);

            final var m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            final var random = new Random();
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final var beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final var nedFrame = new NEDFrame(nedPosition, nedC);
                final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final var trueBeforeGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueBeforeGravityKinematics, noErrorsInlier, random);

                final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final var oldNedFrame = new NEDFrame(nedFrame);
                final var newNedFrame = new NEDFrame();
                final var oldEcefFrame = new ECEFFrame();
                final var newEcefFrame = new ECEFFrame();
                var oldRoll = roll - deltaRoll;
                var oldPitch = pitch - deltaPitch;
                var oldYaw = yaw - deltaYaw;

                final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();
                final var sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (var j = 0; j < m; j++) {
                    final var newRoll = oldRoll + deltaRoll;
                    final var newPitch = oldPitch + deltaPitch;
                    final var newYaw = oldYaw + deltaYaw;
                    final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
                    final var newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers && randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsOutlier, random);
                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                    }

                    final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(trueKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(measuredKinematics,
                            timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final var afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final var trueAfterGravityKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                        TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueAfterGravityKinematics, noErrorsInlier, random);

                final var afterMeanFx = measuredAfterGravityKinematics.getFx();
                final var afterMeanFy = measuredAfterGravityKinematics.getFy();
                final var afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final var calibrator = new PROSACRobustEasyGyroscopeCalibrator(qualityScores, sequences,
                    false, true, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);
            assertEquals(0, calibrateNextIteration);
            assertEquals(0, calibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustEasyGyroscopeCalibrator calibrator) {
        checkLocked((PROSACRobustEasyGyroscopeCalibrator) calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustEasyGyroscopeCalibrator calibrator) {
        checkLocked((PROSACRobustEasyGyroscopeCalibrator) calibrator);
        calibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(final RobustEasyGyroscopeCalibrator calibrator, final int iteration) {
        checkLocked((PROSACRobustEasyGyroscopeCalibrator) calibrator);
        calibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(final RobustEasyGyroscopeCalibrator calibrator, final float progress) {
        checkLocked((PROSACRobustEasyGyroscopeCalibrator) calibrator);
        calibrateProgressChange++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
        calibrateNextIteration = 0;
        calibrateProgressChange = 0;
    }

    private static void checkLocked(final PROSACRobustEasyGyroscopeCalibrator calibrator) {
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
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0));
        assertThrows(LockedException.class, () -> calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0));
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
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactors(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialBias((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialMg(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialGg(null));
        assertThrows(LockedException.class, () -> calibrator.setSequences(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(false));
        assertThrows(LockedException.class, () -> calibrator.setGDependentCrossBiasesEstimated(false));
        assertThrows(LockedException.class, () -> calibrator.setListener(null));
        assertThrows(LockedException.class, () -> calibrator.setProgressDelta(0.0f));
        assertThrows(LockedException.class, () -> calibrator.setConfidence(0.0));
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(1));
        assertThrows(LockedException.class, () -> calibrator.setResultRefined(true));
        assertThrows(LockedException.class, () -> calibrator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> calibrator.setPreliminarySubsetSize(1));
        assertThrows(LockedException.class, () -> calibrator.setThreshold(0.5));
        assertThrows(LockedException.class, () -> calibrator.setQualityScores(null));
        assertThrows(LockedException.class, () -> calibrator.setComputeAndKeepInliersEnabled(true));
        assertThrows(LockedException.class, () -> calibrator.setComputeAndKeepResidualsEnabled(true));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix bg, final Matrix mg, final Matrix gg, final RobustEasyGyroscopeCalibrator calibrator) 
            throws WrongSizeException {

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

    private static void assertCovariance(final RobustEasyGyroscopeCalibrator calibrator) {
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

        final var avgStd = (calibrator.getEstimatedBiasXStandardDeviation() +
                calibrator.getEstimatedBiasYStandardDeviation() +
                calibrator.getEstimatedBiasZStandardDeviation()) / 3.0;
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

    private static void checkCommonAxisAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var j = 0; j < 21; j++) {
            final var colIsZero = j == 8 || j == 10 || j == 11;
            for (var i = 0; i < 21; i++) {
                final var rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var i = 0; i < 21; i++) {
            assertNotEquals(0.0, covariance.getElementAt(i, i));
        }
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var j = 0; j < 21; j++) {
            final var colIsZero = j == 8 || j > 9;
            for (var i = 0; i < 21; i++) {
                final var rowIsZero = i == 8 || i > 9;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(21, covariance.getRows());
        assertEquals(21, covariance.getColumns());

        for (var j = 0; j < 21; j++) {
            final var colIsZero = j > 11;
            for (var i = 0; i < 21; i++) {
                final var rowIsZero = i > 11;
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
}
