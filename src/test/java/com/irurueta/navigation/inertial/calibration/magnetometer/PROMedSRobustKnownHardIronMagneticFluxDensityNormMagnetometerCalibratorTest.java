/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest implements
        RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 7000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-2;

    private static final int MEASUREMENT_NUMBER = 700;

    private static final int OUTLIER_PERCENTAGE = 4;

    private static final double THRESHOLD = 1e-9;

    private static final double OUTLIER_ERROR_FACTOR = 700.0;

    private static final int TIMES = 70;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[1]));
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m2));
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmz, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, m4));
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        new double[1]));
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIron,
                        this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        new double[1], this));
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, new double[1]));
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, new double[1], this));
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2));
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2,
                        this));
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2));
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, this));
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1,
                        mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2,
                        mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m4));
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, mm,
                        this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m3,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m4,
                        this));
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m4));
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m4, this));
    }

    @Test
    public void testConstructor23() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0));
    }

    @Test
    public void testConstructor24() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(-1.0, this));
    }

    @Test
    public void testConstructor25() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements));
    }

    @Test
    public void testConstructor26() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, true));
    }

    @Test
    public void testConstructor27() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, hardIron));
    }

    @Test
    public void testConstructor28() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                -1.0, bm));
    }

    @Test
    public void testConstructor29() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, bm, mm));
    }

    @Test
    public void testConstructor30() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, this));
    }

    @Test
    public void testConstructor31() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true));
    }

    @Test
    public void testConstructor32() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, this));
    }

    @Test
    public void testConstructor33() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, hardIron));
    }

    @Test
    public void testConstructor34() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, hardIron, this));
    }

    @Test
    public void testConstructor35() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, hardIron));
    }

    @Test
    public void testConstructor36() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1],
                        this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, hardIron, this));
    }

    @Test
    public void testConstructor37() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm));
    }

    @Test
    public void testConstructor38() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, this));
    }

    @Test
    public void testConstructor39() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm));
    }

    @Test
    public void testConstructor40() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                -1.0, measurements, true, bm, this));
    }

    @Test
    public void testConstructor41() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, mm));
    }

    @Test
    public void testConstructor42() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, mm, this));
    }

    @Test
    public void testConstructor43() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, mm));
    }

    @Test
    public void testConstructor44() throws WrongSizeException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, mm, this));
    }

    @Test
    public void testConstructor45() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements));
    }

    @Test
    public void testConstructor46() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        true));
    }

    @Test
    public void testConstructor47() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        new double[1]));
    }

    @Test
    public void testConstructor48() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6], bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m2));
    }

    @Test
    public void testConstructor49() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6], bm,
                        mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m1,
                        mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m2,
                        mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm,
                        m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm,
                        m4));
    }

    @Test
    public void testConstructor50() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, this));
    }

    @Test
    public void testConstructor51() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true));
    }

    @Test
    public void testConstructor52() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, this));
    }

    @Test
    public void testConstructor53() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, new double[1]));
    }

    @Test
    public void testConstructor54() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, new double[1], this));
    }

    @Test
    public void testConstructor55() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, new double[1]));
    }

    @Test
    public void testConstructor56() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, new double[1], this));
    }

    @Test
    public void testConstructor57() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m2));
    }

    @Test
    public void testConstructor58() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m2, this));
    }

    @Test
    public void testConstructor59() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m2));
    }

    @Test
    public void testConstructor60() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m2, this));
    }

    @Test
    public void testConstructor61() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1,
                        mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2,
                        mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m4));
    }

    @Test
    public void testConstructor62() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, bm, m4, this));
    }

    @Test
    public void testConstructor63() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, bm, m4));
    }

    @Test
    public void testConstructor64() throws WrongSizeException {
        final double[] qualityScores = new double[7];
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        measurements, true, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        measurements, true, bm, m4, this));
    }

    @Test
    public void testConstructor65() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        qualityScores, groundTruthMagneticFluxDensityNorm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0));
    }

    @Test
    public void testConstructor66() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, this));
    }

    @Test
    public void testConstructor67() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, new double[3], 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, new Matrix(3, 1));
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements));
    }

    @Test
    public void testConstructor68() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, true));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, true));
    }

    @Test
    public void testConstructor69() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, hardIron));
    }

    @Test
    public void testConstructor70() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, bm));
    }

    @Test
    public void testConstructor71() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        qualityScores, groundTruthMagneticFluxDensityNorm, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, bm, mm));
    }

    @Test
    public void testConstructor72() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, this));
    }

    @Test
    public void testConstructor73() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true));
    }

    @Test
    public void testConstructor74() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, this));
    }

    @Test
    public void testConstructor75() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, hardIron));
    }

    @Test
    public void testConstructor76() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, hardIron, this));
    }

    @Test
    public void testConstructor77() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, hardIron));
    }

    @Test
    public void testConstructor78() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1],
                        this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, hardIron, this));
    }

    @Test
    public void testConstructor79() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm));
    }

    @Test
    public void testConstructor80() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, this));
    }

    @Test
    public void testConstructor81() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm));
    }

    @Test
    public void testConstructor82() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, this));
    }

    @Test
    public void testConstructor83() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, mm));
    }

    @Test
    public void testConstructor84() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, mm, this));
    }

    @Test
    public void testConstructor85() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, mm));
    }

    @Test
    public void testConstructor86() throws WrongSizeException, IOException {
        final double[] qualityScores = new double[7];
        final UniformRandomizer randomizer = new UniformRandomizer();
        final NEDPosition position = createPosition(randomizer);
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix bm = Matrix.newFromArray(hardIron);
        final double bmx = hardIron[0];
        final double bmy = hardIron[1];
        final double bmz = hardIron[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);

        // check default values
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(bmx, calibrator.getHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final double[] b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final double[] b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final Matrix bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final MagneticFluxDensity mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final MagneticFluxDensityTriad bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final MagneticFluxDensityTriad bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final Matrix bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final Matrix mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final Matrix mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedMm());
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
        assertEquals(10, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final MagneticFluxDensity b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[6],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, mm, this));
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);

        // set new value
        calibrator.setStopThreshold(THRESHOLD);

        // check
        assertEquals(THRESHOLD, calibrator.getStopThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setStopThreshold(0.0));
    }

    @Test
    public void testGetSetGroundTruthMagneticFluxDensityNorm1() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity norm1 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double groundTruthMagneticFluxDensity = randomizer.nextDouble();
        calibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensity);

        // check
        final Double value = calibrator.getGroundTruthMagneticFluxDensityNorm();
        assertEquals(groundTruthMagneticFluxDensity, value, 0.0);

        final MagneticFluxDensity norm2 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensity, norm2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm2.getUnit());
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));
        assertEquals(norm1, norm2);

        // set new value
        calibrator.setGroundTruthMagneticFluxDensityNorm((Double) null);

        // check
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(-1.0));
    }

    @Test
    public void testGetSetGroundTruthMagneticFluxDensityNorm2() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final MagneticFluxDensity norm1 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer();
        final double groundTruthMagneticFluxDensity = randomizer.nextDouble();
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(groundTruthMagneticFluxDensity,
                MagneticFluxDensityUnit.TESLA);
        calibrator.setGroundTruthMagneticFluxDensityNorm(norm2);

        // check
        final Double value = calibrator.getGroundTruthMagneticFluxDensityNorm();
        assertEquals(groundTruthMagneticFluxDensity, value, 0.0);

        final MagneticFluxDensity norm3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensity, norm3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm3.getUnit());
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));
        assertEquals(norm1, norm3);

        // set new value
        calibrator.setGroundTruthMagneticFluxDensityNorm((MagneticFluxDensity) null);

        // check
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // Force IllegalArgumentException
        final var b = new MagneticFluxDensity(-1.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(b));
    }

    @Test
    public void testGetSetHardIronX() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
    }

    @Test
    public void testGetSetHardIronY() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronY = hardIron[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
    }

    @Test
    public void testGetSetHardIronZ() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronZ = hardIron[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    public void testGetSetHardIronXAsMagneticFluxDensity() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(hardIronX, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronX(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronXAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetHardIronYAsMagneticFluxDensity() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronY = mb[1];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(hardIronY, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronY(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronYAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetHardIronZAsMagneticFluxDensity() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronZ = mb[2];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(hardIronZ, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronZ(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronZAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testSetHardIronCoordinates1() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    public void testSetHardIronCoordinates2() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        final MagneticFluxDensity def = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);

        // check default value
        assertEquals(def, calibrator.getHardIronXAsMagneticFluxDensity());
        assertEquals(def, calibrator.getHardIronYAsMagneticFluxDensity());
        assertEquals(def, calibrator.getHardIronZAsMagneticFluxDensity());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mb = generateHardIron(randomizer);
        final MagneticFluxDensity hardIronX = new MagneticFluxDensity(mb[0], MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity hardIronY = new MagneticFluxDensity(mb[1], MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity hardIronZ = new MagneticFluxDensity(mb[2], MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronXAsMagneticFluxDensity());
        assertEquals(hardIronY, calibrator.getHardIronYAsMagneticFluxDensity());
        assertEquals(hardIronZ, calibrator.getHardIronZAsMagneticFluxDensity());
    }

    @Test
    public void testGetSetHardIronAsTriad() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        final MagneticFluxDensityTriad triad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final MagneticFluxDensityTriad triad3 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                hardIronX, hardIronY, hardIronZ);
        calibrator.setHardIron(triad3);

        final MagneticFluxDensityTriad triad4 = calibrator.getHardIronAsTriad();
        final MagneticFluxDensityTriad triad5 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    public void testGetSetInitialSx() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sy = mm.getElementAt(1, 1);
        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sz = mm.getElementAt(2, 2);
        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);
        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxz = mm.getElementAt(0, 2);
        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myx = mm.getElementAt(1, 0);
        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myz = mm.getElementAt(1, 2);
        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzx = mm.getElementAt(2, 0);
        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzy = mm.getElementAt(2, 1);
        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

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
    public void testSetInitialScalingFactorsAndCrossCouplingErrors() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

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
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

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
    public void testGetSetHardIron() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron2 = generateHardIron(randomizer);
        calibrator.setHardIron(hardIron2);

        // check
        assertArrayEquals(hardIron2, calibrator.getHardIron(), 0.0);
        final double[] hardIron3 = new double[3];
        calibrator.getHardIron(hardIron3);
        assertArrayEquals(hardIron2, hardIron3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIron(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(new double[1]));
    }

    @Test
    public void testGetSetHardIronMatrix() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final Matrix hardIron1 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron1);
        assertEquals(new Matrix(3, 1), hardIron1);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Matrix hardIron2 = Matrix.newFromArray(generateHardIron(randomizer));
        calibrator.setHardIron(hardIron2);

        // check
        assertEquals(calibrator.getHardIronMatrix(), hardIron2);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIronMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIronMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(m4));
    }

    @Test
    public void testGetSetInitialMm() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check initial value
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final Matrix mm1 = new Matrix(3, 3);
        calibrator.getInitialMm(mm1);
        assertEquals(new Matrix(3, 3), mm1);

        // set new value
        final Matrix mm2 = generateSoftIronGeneral();
        calibrator.setInitialMm(mm2);

        // check
        assertEquals(mm2, calibrator.getInitialMm());
        final Matrix mm3 = new Matrix(3, 3);
        calibrator.getInitialMm(mm3);
        assertEquals(mm2, mm3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMm(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialMm(m2));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMm(m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialMm(m4));
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationBodyMagneticFluxDensity> measurements = Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    public void testGetMinimumRequiredMeasurements() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsReady() throws LockedException, IOException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // initially there are no measurements
        assertFalse(calibrator.isReady());
        assertNull(calibrator.getMeasurements());

        // set not enough measurements
        final List<StandardDeviationBodyMagneticFluxDensity> measurements1 = Collections.emptyList();
        calibrator.setMeasurements(measurements1);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final NEDPosition position = createPosition(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIronGeneral();
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final List<StandardDeviationBodyMagneticFluxDensity> measurements2 = generateMeasures(hardIron, softIron,
                calibrator.getMinimumRequiredMeasurements(), wmmEstimator, randomizer, position, timestamp);
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        // set ground truth magnetic flux density norm
        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        calibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);

        // check
        assertFalse(calibrator.isReady());

        // set quality scores
        calibrator.setQualityScores(new double[measurements2.size()]);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

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
    public void testGetSetConfidence() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

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
    public void testGetSetMaxIterations() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(5000, calibrator.getMaxIterations());

        // set new value
        calibrator.setMaxIterations(70);

        assertEquals(70, calibrator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[calibrator.getMinimumRequiredMeasurements()];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, calibrator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setQualityScores(new double[6]));
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(
                PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());

        // set new value
        calibrator.setPreliminarySubsetSize(11);

        // check
        assertEquals(11, calibrator.getPreliminarySubsetSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(6));
    }

    @Test
    public void testCalibrateGeneralNoNoiseInlier() throws IOException, LockedException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 70) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                    error = Math.abs(noiseRandomizer.nextDouble());
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                    error = 0.0;
                }
                measurements.add(b);

                qualityScores[i] = 1.0 / (1.0 + error);
            }
            final GregorianCalendar calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year)
                    .getNorm();

            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                            groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm,
                            this);
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

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    public void testCalibrateCommonAxisNoNoiseInlier() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 70) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                    error = Math.abs(noiseRandomizer.nextDouble());
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                    error = 0.0;
                }
                measurements.add(b);

                qualityScores[i] = 1.0 / (1.0 + error);
            }
            final GregorianCalendar calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                            groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm,
                            this);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    public void testCalibrateGeneralWithInlierNoise() throws IOException, LockedException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < 2 * TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 70) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, outlierNoiseRandomizer, position, timestamp, cnb);
                    error = Math.abs(outlierNoiseRandomizer.nextDouble());
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, inlierNoiseRandomizer, position, timestamp, cnb);
                    error = 0.0;
                }
                measurements.add(b);

                qualityScores[i] = 1.0 / (1.0 + error);
            }
            final GregorianCalendar calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                            groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm,
                            this);
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

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    public void testCalibrateCommonAxisWithInlierNoise() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 70) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, outlierNoiseRandomizer, position, timestamp, cnb);
                    error = Math.abs(outlierNoiseRandomizer.nextDouble());
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, inlierNoiseRandomizer, position, timestamp, cnb);
                    error = 0.0;
                }
                measurements.add(b);

                qualityScores[i] = 1.0 / (1.0 + error);
            }
            final GregorianCalendar calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                            groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm,
                            this);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, mCalibrateStart);
            assertEquals(0, mCalibrateEnd);
            assertEquals(0, mCalibrateNextIteration);
            assertEquals(0, mCalibrateProgressChange);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    public void testCalibrateGeneralNoRefinement() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix bm = Matrix.newFromArray(hardIron);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final NEDPosition position = createPosition(randomizer);
            final Date timestamp = new Date(createTimestamp(randomizer));
            final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final CoordinateTransformation cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 70) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                    error = Math.abs(noiseRandomizer.nextDouble());
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                    error = 0.0;
                }
                measurements.add(b);

                qualityScores[i] = 1.0 / (1.0 + error);
            }
            final GregorianCalendar calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final double year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final double groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year)
                    .getNorm();

            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                            groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm,
                            this);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setResultRefined(false);

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

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, mCalibrateStart);
            assertEquals(1, mCalibrateEnd);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
        checkLocked((PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
        checkLocked((PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private static void checkLocked(
            final PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setStopThreshold(0.0));
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(1.0));
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(
                new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)));
        assertThrows(LockedException.class, () -> calibrator.setHardIronX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setHardIronY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setHardIronZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setHardIronX(null));
        assertThrows(LockedException.class, () -> calibrator.setHardIronY(null));
        assertThrows(LockedException.class, () -> calibrator.setHardIronZ(null));
        assertThrows(LockedException.class, () -> calibrator.setHardIronCoordinates(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setHardIronCoordinates(
                null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setHardIron((MagneticFluxDensityTriad) null));
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
        assertThrows(LockedException.class, () -> calibrator.setHardIron((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setHardIron((Matrix) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialMm(null));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(null));
        assertThrows(LockedException.class, () -> calibrator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> calibrator.setConfidence(0.8));
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(70));
        assertThrows(LockedException.class, () -> calibrator.setResultRefined(true));
        assertThrows(LockedException.class, () -> calibrator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> calibrator.setPreliminarySubsetSize(7));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix mm, final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {

        assertEquals(mm.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(mm.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(mm.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(mm.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(mm.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(mm.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(mm.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(mm.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(mm.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);
    }

    private static void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(9, covariance.getRows());
        assertEquals(9, covariance.getColumns());

        for (int j = 0; j < 9; j++) {
            final boolean colIsZero = j == 5 || j == 7 || j == 8;
            for (int i = 0; i < 9; i++) {
                final boolean rowIsZero = i == 5 || i == 7 || i == 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(0.0, covariance.getElementAt(i, j), 0.0);
                }
            }
        }
    }

    private static void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(9, covariance.getRows());
        assertEquals(9, covariance.getColumns());

        for (int i = 0; i < 9; i++) {
            assertNotEquals(covariance.getElementAt(i, i), 0.0);
        }
    }

    private static NEDMagneticFluxDensity getMagneticFluxDensityAtPosition(
            final NEDPosition position, final double year) throws IOException {
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        return wmmEstimator.estimate(position, year);
    }

    private static List<StandardDeviationBodyMagneticFluxDensity> generateMeasures(
            final double[] hardIron, final Matrix softIron, final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final UniformRandomizer randomizer,
            final NEDPosition position, final Date timestamp) {

        final List<StandardDeviationBodyMagneticFluxDensity> result = new ArrayList<>();
        for (int i = 0; i < numberOfMeasurements; i++) {
            final CoordinateTransformation cnb = generateBodyC(randomizer);
            result.add(generateMeasure(hardIron, softIron, wmmEstimator, null, position, timestamp,
                    cnb));
        }
        return result;
    }

    private static StandardDeviationBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final NEDPosition position, final Date timestamp,
            final CoordinateTransformation cnb) {

        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(position, timestamp);

        final BodyMagneticFluxDensity truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final BodyMagneticFluxDensity measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron,
                softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        final double std = noiseRandomizer != null ? noiseRandomizer.getStandardDeviation() : MAGNETOMETER_NOISE_STD;
        return new StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, std);
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static Matrix generateSoftIronCommonAxis() {
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        for (int col = 0; col < mm.getColumns(); col++) {
            for (int row = 0; row < mm.getRows(); row++) {
                if (row > col) {
                    mm.setElementAt(row, col, 0.0);
                }
            }
        }
        return mm;
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}