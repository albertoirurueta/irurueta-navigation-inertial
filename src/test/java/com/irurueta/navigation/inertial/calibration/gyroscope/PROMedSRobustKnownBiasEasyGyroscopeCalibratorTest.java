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
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSRobustKnownBiasEasyGyroscopeCalibratorTest implements
        RobustKnownBiasEasyGyroscopeCalibratorListener {

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

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, new double[3], 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, new Matrix(3, 1));
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertNull(calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bg, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m6));
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bg, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m6, this));
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bias, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m4));
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bias, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m4, this));
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, gg, baArray, m6));
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                new double[1], mg, gg, baArray, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m1, gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, m2, gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bias, mg, gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bias, mg, gg, baArray, m6, this));
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baZ, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, bg, mg, gg, ba, m10, this));
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bg, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6));
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bg, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, this));
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bias, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4));
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bias, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, this));
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                sequences, false, false, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m6));
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, new double[1], mg, gg, baArray, ma,
                this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m1, gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, m2, gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bias, mg, gg, baArray, m6, this));
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, false,
                        false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences,
                false, false, bg, mg, gg, ba, m10, this));
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator = new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, biasTriad1.getValueX(), 0.0);
        assertEquals(0.0, biasTriad1.getValueY(), 0.0);
        assertEquals(0.0, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(new double[3], bg1, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, new Matrix(3, 1));
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertNull(calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1]));
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m6));
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bg, mg, gg, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m1, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m2, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m3, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m4, gg, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m6, this));
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3],  ba1,0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, new double[1], mg, gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m4));
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bias, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, new double[1], mg, gg, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m1, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m2, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m4, this));
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES, 
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bias, mg, gg, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, new double[1], mg, gg, baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m1, gg, baArray, ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m2, gg, baArray, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m3, baArray, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m4, baArray, ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, baArray, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, baArray, m6));
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bias, mg, gg, baArray, ma,
                        this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bias, mg, gg, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, new double[1], mg, gg, baArray, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m1, gg, baArray, ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, m2, gg, baArray, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m3, baArray, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, baArray, m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bias, mg, gg, baArray, m6, this));
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, bg, mg, gg, ba, ma,
                        this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, bg, mg, gg, ba, ma, this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m1, mg, gg, ba, ma, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, m2, mg, gg, ba, ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m3, gg, ba, ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, m4, gg, ba, ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m5, ba, ma, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, m6, ba, ma, this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, m7, ma, this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, m8, ma, this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, ba, m9, this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, bg, mg, gg, ba, m10, this));
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bg, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m1, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m2, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m3, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m4, gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m6));
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bg, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bg, mg, gg,
                this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m1, mg, gg,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m2, mg, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m3, gg,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m4, gg,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m5,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m6,
                this));
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bias, mg, gg);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bias, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, new double[1], mg,
                gg));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m4));
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bias, mg, gg, this);

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
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bias, mg, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, new double[1], mg, gg,
                this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m1, gg,
                this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m2, gg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m3,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m4,
                this));
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bias, mg, gg, baArray,
                ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, new double[1], mg, gg,
                baArray, ma));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m1, gg, baArray,
                ma));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m2, gg, baArray,
                ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m3, baArray,
                ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m4, baArray,
                ma));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg,
                new double[1], ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg, baArray,
                m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg, baArray,
                m6));
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bias, mg, gg, baArray,
                ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, new double[1], mg, gg,
                baArray, ma, this));
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m1, gg, baArray,
                ma, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, m2, gg, baArray,
                ma, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, m3, baArray,
                ma, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg,
                m4, baArray, ma, this));
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg,
                new double[1], ma, this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg, baArray,
                m5, this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bias, mg, gg, baArray,
                m6, this));
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy,  angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bg, mg, gg, ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m1, mg, gg, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m2, mg, gg, ba, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m3, gg, ba, ma));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m4, gg, ba, ma));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m5, ba, ma));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m6, ba, ma));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, m7, ma));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, m8, ma));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, ba, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, ba, m10));
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final double[] qualityScores = new double[10];
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
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
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                        false, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(baX, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(baY, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baZ, calibrator.getAccelerometerBiasZ(), 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(baX, accelerationX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationX1.getUnit());
        final Acceleration accelerationX2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(baY, accelerationY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationY1.getUnit());
        final Acceleration accelerationY2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(baZ, accelerationZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationZ1.getUnit());
        final Acceleration accelerationZ2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
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
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);

        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bgx, angularSpeedX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedX1.getUnit());
        final AngularSpeed angularSpeedX2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(bgy, angularSpeedY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedY1.getUnit());
        final AngularSpeed angularSpeedY2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bgz, angularSpeedZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedZ1.getUnit());
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(bgx, biasTriad1.getValueX(), 0.0);
        assertEquals(bgy, biasTriad1.getValueY(), 0.0);
        assertEquals(bgz, biasTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, biasTriad1.getUnit());
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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

        final double[] bg1 = calibrator.getBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertSame(sequences, calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES,
                calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());

        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                new double[1], sequences, false, false, bg, mg, gg, ba, ma,
                this));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m1, mg, gg, ba, ma,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, m2, mg, gg, ba, ma,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m3, gg, ba, ma,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, m4, gg, ba, ma,
                this));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m5, ba, ma,
                this));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, m6, ba, ma,
                this));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, m7, ma,
                this));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, m8, ma,
                this));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, ba, m9,
                this));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                qualityScores, sequences, false, false, bg, mg, gg, ba, m10,
                this));
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);

        // set new value
        calibrator.setStopThreshold(1.0);

        // check
        assertEquals(1.0, calibrator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setStopThreshold(0.0));
    }

    @Test
    public void testGetSetAccelerometerBiasX() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final Acceleration bax1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final Acceleration bax3 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(bax3);

        // check
        final Acceleration bax4 = calibrator.getAccelerometerBiasXAsAcceleration();
        final Acceleration bax5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax5);
        assertEquals(bax3, bax4);
        assertEquals(bax3, bax5);
    }

    @Test
    public void testGetSetAccelerometerBiasYAsAcceleration() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final Acceleration bay1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        final Acceleration bay3 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(bay3);

        // check
        final Acceleration bay4 = calibrator.getAccelerometerBiasYAsAcceleration();
        final Acceleration bay5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay5);
        assertEquals(bay3, bay4);
        assertEquals(bay3, bay5);
    }

    @Test
    public void testGetSetAccelerometerBiasZAsAcceleration() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final Acceleration baz1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        final Acceleration baz3 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(baz3);

        // check
        final Acceleration baz4 = calibrator.getAccelerometerBiasZAsAcceleration();
        final Acceleration baz5 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz5);
        assertEquals(baz3, baz4);
        assertEquals(baz3, baz5);
    }

    @Test
    public void testSetAccelerometerBias1() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final Acceleration bax1 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay1 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz1 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(bax1, bay1, baz1);

        // check
        assertEquals(bax, calibrator.getAccelerometerBiasX(), 0.0);
        assertEquals(bay, calibrator.getAccelerometerBiasY(), 0.0);
        assertEquals(baz, calibrator.getAccelerometerBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBias() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final double[] ba1 = calibrator.getAccelerometerBias();
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);

        assertArrayEquals(new double[3], ba1, 0.0);
        assertArrayEquals(ba1, ba2, 0.0);

        // set new value
        final double[] ba3 = generateBa().getBuffer();
        calibrator.setAccelerometerBias(ba3);

        // check
        final double[] ba4 = calibrator.getAccelerometerBias();
        final double[] ba5 = new double[3];
        calibrator.getAccelerometerBias(ba5);

        assertArrayEquals(ba3, ba4, 0.0);
        assertArrayEquals(ba3, ba5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getAccelerometerBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setAccelerometerBias(new double[1]));
    }

    @Test
    public void testGetSetAccelerometerBiasAsMatrix() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);

        // check
        assertEquals(ba1, new Matrix(3, 1));
        assertEquals(ba1, ba2);

        // set new value
        final Matrix ba3 = generateBa();
        calibrator.setAccelerometerBias(ba3);

        final Matrix ba4 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba5 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba5);

        assertEquals(ba3, ba4);
        assertEquals(ba3, ba5);
    }

    @Test
    public void testGetSetAccelerometerSx() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sxa = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(sxa);

        // check
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerSy() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sya = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(sya);

        // check
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerSz() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(sza);

        // check
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxy() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mxya = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(mxya);

        // check
        assertEquals(mxya, calibrator.getAccelerometerMxy(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxz() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mxza = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(mxza);

        // check
        assertEquals(mxza, calibrator.getAccelerometerMxz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyx() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double myxa = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(myxa);

        // check
        assertEquals(myxa, calibrator.getAccelerometerMyx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyz() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator = new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double myza = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(myza);

        // check
        assertEquals(myza, calibrator.getAccelerometerMyz(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzx() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mzxa = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(mzxa);

        // check
        assertEquals(mzxa, calibrator.getAccelerometerMzx(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzy() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(mzya);

        // check
        assertEquals(mzya, calibrator.getAccelerometerMzy(), 0.0);
    }

    @Test
    public void testSetAccelerometerScalingFactors() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerSz(), 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(sxa, sya, sza);

        // check
        assertEquals(sxa, calibrator.getAccelerometerSx(), 0.0);
        assertEquals(sya, calibrator.getAccelerometerSy(), 0.0);
        assertEquals(sza, calibrator.getAccelerometerSz(), 0.0);
    }

    @Test
    public void testSetAccelerometerCrossCouplingErrors() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getAccelerometerMxy(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMxz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMyz(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzx(), 0.0);
        assertEquals(0.0, calibrator.getAccelerometerMzy(), 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

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
    public void testSetAccelerometerScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

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
    public void testGetSetAccelerometerMa() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        assertEquals(new Matrix(3, 3), calibrator.getAccelerometerMa());
        final Matrix ma1 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma1);
        assertEquals(ma1, new Matrix(3, 3));

        // set new value
        final Matrix ma2 = generateMa();
        calibrator.setAccelerometerMa(ma2);

        // check
        assertEquals(ma2, calibrator.getAccelerometerMa());
        final Matrix ma3 = new Matrix(3, 3);
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
    public void testGetSetBiasX() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasX(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);

        calibrator.setBiasX(bgx);

        // check
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
    }

    @Test
    public void testGetSetBiasY() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasY(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);

        calibrator.setBiasY(bgy);

        // check
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
    }

    @Test
    public void testGetSetBiasZ() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);

        calibrator.setBiasZ(bgz);

        // check
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetBiasAngularSpeedX() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final AngularSpeed bgx2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final AngularSpeed bgx3 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setBiasX(bgx3);

        // check
        final AngularSpeed bgx4 = calibrator.getBiasAngularSpeedX();
        final AngularSpeed bgx5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bgx5);

        assertEquals(bgx3, bgx4);
        assertEquals(bgx3, bgx5);
    }

    @Test
    public void testGetSetBiasAngularSpeedY() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgy1 = calibrator.getBiasAngularSpeedY();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final AngularSpeed bgy2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);
        final AngularSpeed bgy3 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setBiasY(bgy3);

        // check
        final AngularSpeed bgy4 = calibrator.getBiasAngularSpeedY();
        final AngularSpeed bgy5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(bgy5);

        assertEquals(bgy3, bgy4);
        assertEquals(bgy3, bgy5);
    }

    @Test
    public void testGetSetBiasAngularSpeedZ() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        final AngularSpeed bgz2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeed bgz3 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setBiasZ(bgz3);

        // check
        final AngularSpeed bgz4 = calibrator.getBiasAngularSpeedZ();
        final AngularSpeed bgz5 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bgz5);

        assertEquals(bgz3, bgz4);
        assertEquals(bgz3, bgz5);
    }

    @Test
    public void testSetBiasCoordinates1() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        calibrator.setBiasCoordinates(bgx, bgy, bgz);

        // check
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBiasCoordinates2() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getBiasX(), 0.0);
        assertEquals(0.0, calibrator.getBiasY(), 0.0);
        assertEquals(0.0, calibrator.getBiasZ(), 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final AngularSpeed bgx1 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy1 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz1 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasCoordinates(bgx1, bgy1, bgz1);

        // check
        assertEquals(bgx, calibrator.getBiasX(), 0.0);
        assertEquals(bgy, calibrator.getBiasY(), 0.0);
        assertEquals(bgz, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetBiasAsTriad() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        final AngularSpeedTriad triad1 = calibrator.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND);
        triad2.setValueCoordinates(bg);

        calibrator.setBias(triad2);

        // check
        final AngularSpeedTriad triad3 = calibrator.getBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {

        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final Matrix mg = generateGeneralMg();
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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final Matrix mg = generateGeneralMg();
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
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
        final Matrix mg = generateGeneralMg();
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
    public void testGetSetBias() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        final double[] bg1 = calibrator.getBias();
        final double[] bg2 = new double[3];
        calibrator.getBias(bg2);

        assertArrayEquals(bg1, new double[3], 0.0);
        assertArrayEquals(bg1, bg2, 0.0);

        // set new value
        final double[] bg3 = generateBg().getBuffer();
        calibrator.setBias(bg3);

        // check
        final double[] bg4 = calibrator.getBias();
        final double[] bg5 = new double[3];
        calibrator.getBias(bg5);

        assertArrayEquals(bg3, bg4, 0.0);
        assertArrayEquals(bg3, bg5, 0.0);


        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getBias(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setBias(new double[1]));
    }

    @Test
    public void testGetSetBiasAsMatrix() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial values
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);

        assertEquals(bg1, new Matrix(3, 1));
        assertEquals(bg1, bg2);

        // set new value
        final Matrix bg3 = generateBg();
        calibrator.setBias(bg3);

        // check
        final Matrix bg4 = calibrator.getBiasAsMatrix();
        final Matrix bg5 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg5);

        assertEquals(bg3, bg4);
        assertEquals(bg3, bg5);

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
    public void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);

        assertEquals(mg1, new Matrix(3, 3));
        assertEquals(mg1, mg2);

        // set new value
        final Matrix mg3 = generateGeneralMg();
        calibrator.setInitialMg(mg3);

        // check
        final Matrix mg4 = calibrator.getInitialMg();
        final Matrix mg5 = new Matrix(3, 3);
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
    public void testGetSetInitialGg() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);

        assertEquals(gg1, new Matrix(3, 3));
        assertEquals(gg1, gg2);

        // set new value
        final Matrix gg3 = generateGg();
        calibrator.setInitialGg(gg3);

        // check
        final Matrix gg4 = calibrator.getInitialGg();
        final Matrix gg5 = new Matrix(3, 3);
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
    public void testGetSetSequences() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getSequences());

        // set new value
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = Collections.emptyList();
        calibrator.setSequences(sequences);

        // check
        assertSame(calibrator.getSequences(), sequences);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsSetGDependentCrossBiasesEstimated() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void tetGetSetListener() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testGetMinimumRequiredMeasurementsOrSequences() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check initial value
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
    public void testIsReady() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        assertFalse(calibrator.isReady());
        assertEquals(16, calibrator.getMinimumRequiredMeasurementsOrSequences());

        // set empty sequences
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences1 = Collections.emptyList();
        calibrator.setSequences(sequences1);

        // check
        assertFalse(calibrator.isReady());

        // set enough sequences
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences2 = new ArrayList<>();
        for (int i = 0; i < calibrator.getMinimumRequiredMeasurementsOrSequences(); i++) {
            sequences2.add(new BodyKinematicsSequence<>());
        }
        calibrator.setSequences(sequences2);

        // check
        assertFalse(calibrator.isReady());

        // add quality scores with invalid size
        double[] qualityScores = new double[sequences2.size() + 1];
        calibrator.setQualityScores(qualityScores);

        assertFalse(calibrator.isReady());

        // add quality scores with valid size
        qualityScores = new double[sequences2.size()];
        calibrator.setQualityScores(qualityScores);

        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0);

        // set new value
        calibrator.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, calibrator.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(2.0f));
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(), 0.0);

        // set new value
        calibrator.setConfidence(0.8);

        // check
        assertEquals(0.8, calibrator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(2.0));
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertEquals(PROMedSRobustKnownBiasEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());

        // set new value
        calibrator.setMaxIterations(1);

        assertEquals(1, calibrator.getMaxIterations());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, calibrator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setQualityScores(new double[9]));
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();

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
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, true,
                            false, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                            false, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0, angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, true,
                            true, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

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
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                            true, bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

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
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, true,
                            false, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                            false, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, true,
                            true, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            if (calibrator.getEstimatedCovariance() != null) {
                checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            }
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledWithInlierNoise() throws WrongSizeException,
            InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg, 0.0,
                    0.0, accelQuantLevel, gyroQuantLevel);

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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(random, 0.0,
                    angularRateStandardDeviation);

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw,
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueBeforeGravityKinematics, noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_VARIATION_DEGREES,
                        MAX_ANGLE_VARIATION_DEGREES));

                final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                final NEDFrame newNedFrame = new NEDFrame();
                final ECEFFrame oldEcefFrame = new ECEFFrame();
                final ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();
                final boolean sequenceCanHaveOutliers = randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                if (sequenceCanHaveOutliers) {
                    error = Math.abs(errorRandomizer.nextDouble());
                } else {
                    error = 0.0;
                }
                qualityScores[i] = 1.0 / (1.0 + error);

                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
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

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation, angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ,
                        QuaternionStepIntegratorType.RUNGE_KUTTA, afterQ);

                final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(
                        TIME_INTERVAL_SECONDS, trueAfterGravityKinematics, noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator =
                    new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, false,
                            true, bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownBiasEasyGyroscopeCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownBiasEasyGyroscopeCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownBiasEasyGyroscopeCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(final RobustKnownBiasEasyGyroscopeCalibrator calibrator, final int iteration) {
        checkLocked((PROMedSRobustKnownBiasEasyGyroscopeCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasEasyGyroscopeCalibrator calibrator, final float progress) {
        checkLocked((PROMedSRobustKnownBiasEasyGyroscopeCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private static void checkLocked(final PROMedSRobustKnownBiasEasyGyroscopeCalibrator calibrator) {
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
        assertThrows(LockedException.class, () -> calibrator.setBiasX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasX(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasY(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasZ(null));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBiasCoordinates(null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setBias((AngularSpeedTriad) null));
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
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setBias((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setBias((Matrix) null));
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
        assertThrows(LockedException.class, () -> calibrator.setStopThreshold(0.5));
        assertThrows(LockedException.class, () -> calibrator.setQualityScores(null));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(final Matrix mg, final Matrix gg,
                                              final RobustKnownBiasEasyGyroscopeCalibrator calibrator) {

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

        for (int j = 0; j < 18; j++) {
            final boolean colIsZero = j == 5 || j == 7 || j == 8;
            for (int i = 0; i < 18; i++) {
                final boolean rowIsZero = i == 5 || i == 7 || i == 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralAndGDependantCrossBiasesCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (int i = 0; i < 18; i++) {
            assertNotEquals(covariance.getElementAt(i, i), 0.0);
        }
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (int j = 0; j < 18; j++) {
            final boolean colIsZero = j == 5 || j > 6;
            for (int i = 0; i < 18; i++) {
                final boolean rowIsZero = i == 5 || i > 6;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(18, covariance.getRows());
        assertEquals(18, covariance.getColumns());

        for (int j = 0; j < 18; j++) {
            final boolean colIsZero = j > 8;
            for (int i = 0; i < 18; i++) {
                final boolean rowIsZero = i > 8;
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
}
