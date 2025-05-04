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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorTest implements
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
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 5e-2;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 4;

    private static final double THRESHOLD = 1e-9;

    private static final double OUTLIER_ERROR_FACTOR = 1000.0;

    private static final int TIMES = 100;

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

    private int calibrateStart;
    private int calibrateEnd;
    private int calibrateNextIteration;
    private int calibrateProgressChange;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                true);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(new double[1]));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m2));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(bm, m4));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        new double[1]));
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                hardIron, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        new double[1], this));
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, new double[1]));
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, hardIron, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, new double[1], this));
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2));
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                bm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2,
                        this));
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2));
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, this));
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m4));
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                bm, mm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                        m4, this));
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m4));
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm, mm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m4, this));
    }

    @Test
    void testConstructor23() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm);

        // check default values
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0));
    }

    @Test
    void testConstructor24() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, this));
    }

    @Test
    void testConstructor25() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements));
    }

    @Test
    void testConstructor26() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, true);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, true));
    }

    @Test
    void testConstructor27() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, hardIron));
    }

    @Test
    void testConstructor28() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, bm));
    }

    @Test
    void testConstructor29() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, bm, mm));
    }

    @Test
    void testConstructor30() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, this));
    }

    @Test
    void testConstructor31() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true));
    }

    @Test
    void testConstructor32() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, this));
    }

    @Test
    void testConstructor33() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, hardIron));
    }

    @Test
    void testConstructor34() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, hardIron, this));
    }

    @Test
    void testConstructor35() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, hardIron));
    }

    @Test
    void testConstructor36() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1],
                        this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, hardIron,
                        this));
    }

    @Test
    void testConstructor37() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        MagneticFluxDensity mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm));
    }

    @Test
    void testConstructor38() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, this));
    }

    @Test
    void testConstructor39() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm));
    }

    @Test
    void testConstructor40() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(new Matrix(3, 3), mm1);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, this));
    }

    @Test
    void testConstructor41() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, mm));
    }

    @Test
    void testConstructor42() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, bm, mm, this));
    }

    @Test
    void testConstructor43() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD, 
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, mm));
    }

    @Test
    void testConstructor44() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);

        // check default values
        assertEquals(MSACRobustKnownPositionAndInstantMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
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

        final var b1 = calibrator.getHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getHardIronMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(mb1.getValue().doubleValue(), bmx, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
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
        final var bTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(bm2);
        assertEquals(bm1, bm2);
        final var mm1 = calibrator.getInitialMm();
        assertEquals(mm1, mm);
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm1, mm2);

        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
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
        assertEquals(RobustEstimatorMethod.MSAC, calibrator.getMethod());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, mm, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);

        // set a new value
        calibrator.setThreshold(THRESHOLD);

        // check
        assertEquals(THRESHOLD, calibrator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setThreshold(0.0));
    }

    @Test
    void testGetSetGroundTruthMagneticFluxDensityNorm1() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var norm1 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var groundTruthMagneticFluxDensity = randomizer.nextDouble();
        calibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensity);

        // check
        final var value = calibrator.getGroundTruthMagneticFluxDensityNorm();
        assertEquals(groundTruthMagneticFluxDensity, value, 0.0);

        final var norm2 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensity, norm2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm2.getUnit());
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));
        assertEquals(norm1, norm2);

        // set a new value
        calibrator.setGroundTruthMagneticFluxDensityNorm((Double) null);

        // check
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(-1.0));
    }

    @Test
    void testGetSetGroundTruthMagneticFluxDensityNorm2() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var norm1 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var groundTruthMagneticFluxDensity = randomizer.nextDouble();
        final var norm2 = new MagneticFluxDensity(groundTruthMagneticFluxDensity, MagneticFluxDensityUnit.TESLA);
        calibrator.setGroundTruthMagneticFluxDensityNorm(norm2);

        // check
        final var value = calibrator.getGroundTruthMagneticFluxDensityNorm();
        assertEquals(groundTruthMagneticFluxDensity, value, 0.0);

        final var norm3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensity, norm3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm3.getUnit());
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));
        assertEquals(norm1, norm3);

        // set a new value
        calibrator.setGroundTruthMagneticFluxDensityNorm((MagneticFluxDensity) null);

        // check
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // Force IllegalArgumentException
        final var b = new MagneticFluxDensity(-1.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(b));
    }

    @Test
    void testGetSetHardIronX() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
    }

    @Test
    void testGetSetHardIronY() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronY = hardIron[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
    }

    @Test
    void testGetSetHardIronZ() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronZ = hardIron[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    void testGetSetHardIronXAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];
        final var b2 = new MagneticFluxDensity(hardIronX, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronX(b2);

        // check
        final var b3 = calibrator.getHardIronXAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testGetSetHardIronYAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronY = mb[1];
        final var b2 = new MagneticFluxDensity(hardIronY, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronY(b2);

        // check
        final var b3 = calibrator.getHardIronYAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testGetSetHardIronZAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronZ = mb[2];
        final var b2 = new MagneticFluxDensity(hardIronZ, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronZ(b2);

        // check
        final var b3 = calibrator.getHardIronZAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testSetHardIronCoordinates1() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    void testSetHardIronCoordinates2() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        final var def = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);

        // check default value
        assertEquals(def, calibrator.getHardIronXAsMagneticFluxDensity());
        assertEquals(def, calibrator.getHardIronYAsMagneticFluxDensity());
        assertEquals(def, calibrator.getHardIronZAsMagneticFluxDensity());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = new MagneticFluxDensity(mb[0], MagneticFluxDensityUnit.TESLA);
        final var hardIronY = new MagneticFluxDensity(mb[1], MagneticFluxDensityUnit.TESLA);
        final var hardIronZ = new MagneticFluxDensity(mb[2], MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronXAsMagneticFluxDensity());
        assertEquals(hardIronY, calibrator.getHardIronYAsMagneticFluxDensity());
        assertEquals(hardIronZ, calibrator.getHardIronZAsMagneticFluxDensity());
    }

    @Test
    void testGetSetHardIronAsTriad() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        final var triad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final var triad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];
        final var hardIronY = mb[1];
        final var hardIronZ = mb[2];

        final var triad3 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, hardIronX, hardIronY, hardIronZ);
        calibrator.setHardIron(triad3);

        final var triad4 = calibrator.getHardIronAsTriad();
        final var triad5 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    void testGetSetInitialSx() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        calibrator.setInitialSx(sx);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    void testGetSetInitialSy() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sy = mm.getElementAt(1, 1);
        calibrator.setInitialSy(sy);

        // check
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    void testGetSetInitialSz() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sz = mm.getElementAt(2, 2);
        calibrator.setInitialSz(sz);

        // check
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testGetSetInitialMxy() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var mxy = mm.getElementAt(0, 1);
        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    void testGetSetInitialMxz() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var mxz = mm.getElementAt(0, 2);
        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    void testGetSetInitialMyx() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var myx = mm.getElementAt(1, 0);
        calibrator.setInitialMyx(myx);

        // check
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    void testGetSetInitialMyz() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var myz = mm.getElementAt(1, 2);
        calibrator.setInitialMyz(myz);

        // check
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    void testGetSetInitialMzx() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var mzx = mm.getElementAt(2, 0);
        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    void testGetSetInitialMzy() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set a new value
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var mzy = mm.getElementAt(2, 1);
        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    void testSetInitialScalingFactors() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);

        // set new values
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    void testSetInitialCrossCouplingErrors() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set new values
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

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
    void testSetInitialScalingFactorsAndCrossCouplingErrors() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var sx = mm.getElementAt(0, 0);
        final var sy = mm.getElementAt(1, 1);
        final var sz = mm.getElementAt(2, 2);
        final var mxy = mm.getElementAt(0, 1);
        final var mxz = mm.getElementAt(0, 2);
        final var myx = mm.getElementAt(1, 0);
        final var myz = mm.getElementAt(1, 2);
        final var mzx = mm.getElementAt(2, 0);
        final var mzy = mm.getElementAt(2, 1);

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
    void testGetSetHardIron() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron2 = generateHardIron(randomizer);
        calibrator.setHardIron(hardIron2);

        // check
        assertArrayEquals(hardIron2, calibrator.getHardIron(), 0.0);
        final var hardIron3 = new double[3];
        calibrator.getHardIron(hardIron3);
        assertArrayEquals(hardIron2, hardIron3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIron(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(new double[1]));
    }

    @Test
    void testGetSetHardIronAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var hardIron1 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron1);
        assertEquals(new Matrix(3, 1), hardIron1);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron2 = Matrix.newFromArray(generateHardIron(randomizer));
        calibrator.setHardIron(hardIron2);

        // check
        assertEquals(calibrator.getHardIronMatrix(), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
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
    void testGetSetInitialMm() throws WrongSizeException, LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check initial value
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm1 = new Matrix(3, 3);
        calibrator.getInitialMm(mm1);
        assertEquals(new Matrix(3, 3), mm1);

        // set a new value
        final var mm2 = generateSoftIronGeneral();
        calibrator.setInitialMm(mm2);

        // check
        assertEquals(mm2, calibrator.getInitialMm());
        final var mm3 = new Matrix(3, 3);
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
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set a new value
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set a new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetMinimumRequiredMeasurements() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertEquals(7, calibrator.getMinimumRequiredMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testIsReady() throws LockedException, IOException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // initially there are no measurements
        assertFalse(calibrator.isReady());
        assertNull(calibrator.getMeasurements());

        // set not enough measurements
        final var measurements1 = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        calibrator.setMeasurements(measurements1);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIronGeneral();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var measurements2 = generateMeasures(hardIron, softIron, calibrator.getMinimumRequiredMeasurements(), 
                wmmEstimator, randomizer, position, timestamp);
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        // set ground truth magnetic flux density norm
        final var calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        calibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.05f, calibrator.getProgressDelta(), 0.0);

        // set a new value
        calibrator.setProgressDelta(0.01f);

        // check
        assertEquals(0.01f, calibrator.getProgressDelta(), 0.0);

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setProgressDelta(2.0f));
    }

    @Test
    void testGetSetConfidence() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.99, calibrator.getConfidence(), 0.0);

        // set a new value
        calibrator.setConfidence(0.5);

        // check
        assertEquals(0.5, calibrator.getConfidence(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(-1.0));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setConfidence(2.0));
    }

    @Test
    void testGetSetMaxIterations() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(5000, calibrator.getMaxIterations());

        // set a new value
        calibrator.setMaxIterations(100);

        assertEquals(100, calibrator.getMaxIterations());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setMaxIterations(0));
    }

    @Test
    void testIsSetResultRefined() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set a new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set a new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set a new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());

        // set a new value
        calibrator.setPreliminarySubsetSize(11);

        // check
        assertEquals(11, calibrator.getPreliminarySubsetSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(6));
    }

    @Test
    void testCalibrateGeneralNoNoiseInlier() throws IOException, LockedException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var bm = Matrix.newFromArray(hardIron);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                }
                measurements.add(b);
            }
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm, this);
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

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    void testCalibrateCommonAxisNoNoiseInlier() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var bm = Matrix.newFromArray(hardIron);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                }
                measurements.add(b);
            }
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);
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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    void testCalibrateGeneralWithInlierNoise() throws IOException, LockedException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < 2 * TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var bm = Matrix.newFromArray(hardIron);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var inlierNoiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);
            final var outlierNoiseRandomizer = new GaussianRandomizer(0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, outlierNoiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, inlierNoiseRandomizer, position, timestamp, cnb);
                }
                measurements.add(b);
            }
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm, this);
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

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);
            assertTrue(calibrateNextIteration > 0);
            assertTrue(calibrateProgressChange >= 0);

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    void testCalibrateCommonAxisWithInlierNoise() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var bm = Matrix.newFromArray(hardIron);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var inlierNoiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);
            final var outlierNoiseRandomizer = new GaussianRandomizer(0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, outlierNoiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, inlierNoiseRandomizer, position, timestamp, cnb);
                }
                measurements.add(b);
            }
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);
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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

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
    void testCalibrateGeneralNoRefinement() throws IOException, LockedException, CalibrationException,
            NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var bm = Matrix.newFromArray(hardIron);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, noiseRandomizer, position, timestamp, cnb);
                } else {
                    // inlier
                    b = generateMeasure(hardIron, mm, wmmEstimator, null, position, timestamp, cnb);
                }
                measurements.add(b);
            }
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, false, bm, mm, this);
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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() >= 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
        checkLocked((MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
        checkLocked((MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateProgressChange++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
        calibrateNextIteration = 0;
        calibrateProgressChange = 0;
    }

    private static void checkLocked(
            final MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setThreshold(0.0));
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
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(100));
        assertThrows(LockedException.class, () -> calibrator.setResultRefined(true));
        assertThrows(LockedException.class, () -> calibrator.setCovarianceKept(true));
        assertThrows(LockedException.class, () -> calibrator.setPreliminarySubsetSize(10));
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

    private static NEDMagneticFluxDensity getMagneticFluxDensityAtPosition(
            final NEDPosition position, final double year) throws IOException {
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        return wmmEstimator.estimate(position, year);
    }

    private static List<StandardDeviationBodyMagneticFluxDensity> generateMeasures(
            final double[] hardIron, final Matrix softIron,
            final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final NEDPosition position,
            final Date timestamp) {

        final var result = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
        for (var i = 0; i < numberOfMeasurements; i++) {
            final var cnb = generateBodyC(randomizer);
            result.add(generateMeasure(hardIron, softIron, wmmEstimator, null, position, timestamp, 
                    cnb));
        }
        return result;
    }

    private static StandardDeviationBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final NEDPosition position, final Date timestamp,
            final CoordinateTransformation cnb) {

        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        final var std = noiseRandomizer != null ? noiseRandomizer.getStandardDeviation() : MAGNETOMETER_NOISE_STD;
        return new StandardDeviationBodyMagneticFluxDensity(measuredMagnetic, std);
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
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
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        for (var col = 0; col < mm.getColumns(); col++) {
            for (var row = 0; row < mm.getRows(); row++) {
                if (row > col) {
                    mm.setElementAt(row, col, 0.0);
                }
            }
        }
        return mm;
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}