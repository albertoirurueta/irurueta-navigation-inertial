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

class PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibratorTest implements
        RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener {

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
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[1]));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(m2));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(bm, m4));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator =
                new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, new double[1]));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, new double[1],
                        this));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, true,
                        new double[1]));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, true,
                        new double[1], this));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm,
                this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2,
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                true, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m4));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, mm,
                this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm,
                        this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm,
                        this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m3,
                        this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m4,
                        this));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                        true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements,
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(-1.0));
    }

    @Test
    void testConstructor24() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(-1.0, true));
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
        final var calibrator =
                new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(groundTruthMagneticFluxDensityNorm,
                        measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator =
                new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(groundTruthMagneticFluxDensityNorm,
                        measurements, true, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1],
                        this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
                        -1.0, measurements, true, bm, mm, this));
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements));
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        true));
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, new double[1]));
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m2));
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm,
                mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, bm, m4));
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        this));
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true));
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, this));
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        new double[1]));
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        new double[1], this));
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, new double[1]));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, new double[1], this));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m2));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m2, this));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m2));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var qualityScores = new double[10];
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var bm = Matrix.newFromArray(hardIron);
        final var bmx = hardIron[0];
        final var bmy = hardIron[1];
        final var bmz = hardIron[2];

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m2, this));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, bm, m4));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        bm, m4, this));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, m4));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                measurements, true, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9], measurements,
                        true, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores, measurements,
                        true, bm, m4, this));
    }

    @Test
    void testConstructor65() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                calibrator.getConfidence(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
                calibrator.getMaxIterations());
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0));
    }

    @Test
    void testConstructor66() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, this));
    }

    @Test
    void testConstructor67() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements));
    }

    @Test
    void testConstructor68() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, true));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, true));
    }

    @Test
    void testConstructor69() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, hardIron));
    }

    @Test
    void testConstructor70() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, bm));
    }

    @Test
    void testConstructor71() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, bm, mm));
    }

    @Test
    void testConstructor72() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, this));
    }

    @Test
    void testConstructor73() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true));
    }

    @Test
    void testConstructor74() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(new double[3], b1, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(new Matrix(3, 1), bm1);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, this));
    }

    @Test
    void testConstructor75() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, hardIron));
    }

    @Test
    void testConstructor76() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, hardIron, this));
    }

    @Test
    void testConstructor77() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, hardIron));
    }

    @Test
    void testConstructor78() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, new double[1],
                        this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, hardIron, this));
    }

    @Test
    void testConstructor79() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm));
    }

    @Test
    void testConstructor80() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, this));
    }

    @Test
    void testConstructor81() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, bm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm));
    }

    @Test
    void testConstructor82() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
        assertEquals(bm1, bm2);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, this));
    }

    @Test
    void testConstructor83() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, mm));
    }

    @Test
    void testConstructor84() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, bm, mm, this));
    }

    @Test
    void testConstructor85() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, mm));
    }

    @Test
    void testConstructor86() throws WrongSizeException, IOException {
        final var qualityScores = new double[10];
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

        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this);

        // check default values
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertEquals(bmx, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(bmy, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(bmz, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);

        final var b1 = calibrator.getInitialHardIron();
        assertArrayEquals(b1, hardIron, 0.0);
        final var b2 = new double[3];
        calibrator.getInitialHardIron(b2);
        assertArrayEquals(b1, b2, 0.0);
        final var bm1 = calibrator.getInitialHardIronAsMatrix();
        assertEquals(bm1, bm);
        var mb1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(bmx, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        final var mb2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(bmy, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        mb1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(bmz, mb1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, mb1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(mb2);
        assertEquals(mb1, mb2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(bmx, bTriad1.getValueX(), 0.0);
        assertEquals(bmy, bTriad1.getValueY(), 0.0);
        assertEquals(bmz, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        final var bm2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(bm2);
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
        assertTrue(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
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
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedHardIronXAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
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
        assertEquals(13, calibrator.getPreliminarySubsetSize());
        assertEquals(RobustEstimatorMethod.PROSAC, calibrator.getMethod());
        assertNull(calibrator.getEstimatedHardIronXVariance());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronYVariance());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronZVariance());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviation());
        assertNull(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronAsTriad());
        assertFalse(calibrator.getEstimatedHardIronAsTriad(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverage());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(null));
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNorm());
        assertNull(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity());
        assertFalse(calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(null));
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
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(new double[9],
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, mm, this));
        final var m1 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        groundTruthMagneticFluxDensityNorm, measurements, true, bm, m4, this));
        assertThrows(IllegalArgumentException.class,
                () -> new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                        -1.0, measurements, true, bm, mm, this));
    }

    @Test
    void testGetSetThreshold() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_THRESHOLD,
                calibrator.getThreshold(), 0.0);

        // set a new value
        calibrator.setThreshold(THRESHOLD);

        // check
        assertEquals(THRESHOLD, calibrator.getThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setThreshold(0.0));
    }

    @Test
    void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS,
                calibrator.isComputeAndKeepInliersEnabled());
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());

        // set a new value
        calibrator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepInliersEnabled());
    }

    @Test
    void testIsSetComputeAndKeepResiduals() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS,
                calibrator.isComputeAndKeepResiduals());
        assertFalse(calibrator.isComputeAndKeepResiduals());

        // set a new value
        calibrator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepResiduals());
    }

    @Test
    void testGetSetGroundTruthMagneticFluxDensityNorm1() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
    void testGetSetInitialHardIronX() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];

        calibrator.setInitialHardIronX(hardIronX);

        // check
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
    }

    @Test
    void testGetSetInitialHardIronY() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronY = hardIron[1];

        calibrator.setInitialHardIronY(hardIronY);

        // check
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
    }

    @Test
    void testGetSetInitialHardIronZ() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronZ = hardIron[2];

        calibrator.setInitialHardIronZ(hardIronZ);

        // check
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
    }

    @Test
    void testGetSetInitialHardIronXAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];
        final var b2 = new MagneticFluxDensity(hardIronX, MagneticFluxDensityUnit.TESLA);

        calibrator.setInitialHardIronX(b2);

        // check
        final var b3 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testGetSetInitialHardIronYAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronY = mb[1];
        final var b2 = new MagneticFluxDensity(hardIronY, MagneticFluxDensityUnit.TESLA);

        calibrator.setInitialHardIronY(b2);

        // check
        final var b3 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testGetSetInitialHardIronZAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        final var b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronZ = mb[2];
        final var b2 = new MagneticFluxDensity(hardIronZ, MagneticFluxDensityUnit.TESLA);

        calibrator.setInitialHardIronZ(b2);

        // check
        final var b3 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        final var b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    void testSetInitialHardIron1() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getInitialHardIronZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        calibrator.setInitialHardIron(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
    }

    @Test
    void testSetInitialHardIron2() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        final var def = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);

        // check default value
        assertEquals(def, calibrator.getInitialHardIronXAsMagneticFluxDensity());
        assertEquals(def, calibrator.getInitialHardIronYAsMagneticFluxDensity());
        assertEquals(def, calibrator.getInitialHardIronZAsMagneticFluxDensity());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = new MagneticFluxDensity(mb[0], MagneticFluxDensityUnit.TESLA);
        final var hardIronY = new MagneticFluxDensity(mb[1], MagneticFluxDensityUnit.TESLA);
        final var hardIronZ = new MagneticFluxDensity(mb[2], MagneticFluxDensityUnit.TESLA);

        calibrator.setInitialHardIron(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getInitialHardIronXAsMagneticFluxDensity());
        assertEquals(hardIronY, calibrator.getInitialHardIronYAsMagneticFluxDensity());
        assertEquals(hardIronZ, calibrator.getInitialHardIronZAsMagneticFluxDensity());
    }

    @Test
    void testGetSetInitialHardIronAsTriad() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
        final var triad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final var triad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];
        final var hardIronY = mb[1];
        final var hardIronZ = mb[2];

        final var triad3 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, hardIronX, hardIronY, hardIronZ);
        calibrator.setInitialHardIron(triad3);

        final var triad4 = calibrator.getInitialHardIronAsTriad();
        final var triad5 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    void testGetSetInitialSx() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
    void testGetSetInitialHardIron() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getInitialHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron2 = generateHardIron(randomizer);
        calibrator.setInitialHardIron(hardIron2);

        // check
        assertArrayEquals(hardIron2, calibrator.getInitialHardIron(), 0.0);
        final var hardIron3 = new double[3];
        calibrator.getInitialHardIron(hardIron3);
        assertArrayEquals(hardIron2, hardIron3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialHardIron(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialHardIron(new double[1]));
    }

    @Test
    void testGetSetInitialHardIronAsMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIron1 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIron1);
        assertEquals(new Matrix(3, 1), hardIron1);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var hardIron2 = Matrix.newFromArray(generateHardIron(randomizer));
        calibrator.setInitialHardIron(hardIron2);

        // check
        assertEquals(hardIron2, calibrator.getInitialHardIronAsMatrix());
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialHardIronAsMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getInitialHardIronAsMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialHardIron(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setInitialHardIron(m4));
    }

    @Test
    void testGetSetInitialMm() throws WrongSizeException, LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set a new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetMinimumRequiredMeasurements() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(13, calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertEquals(10, calibrator.getMinimumRequiredMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testIsReady() throws LockedException, IOException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        assertFalse(calibrator.isReady());

        // set quality scores
        calibrator.setQualityScores(new double[measurements2.size()]);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set a new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set a new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set a new value
        final var qualityScores = new double[calibrator.getMinimumRequiredMeasurements()];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, calibrator.getQualityScores());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setQualityScores(new double[9]));
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertEquals(PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getPreliminarySubsetSize());

        // set a new value
        calibrator.setPreliminarySubsetSize(11);

        // check
        assertEquals(11, calibrator.getPreliminarySubsetSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(9));
    }

    @Test
    void testCalibrateGeneralNoNoiseInlier() throws IOException, LockedException, WrongSizeException, 
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
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
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
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator, true);

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
            NotReadyException, WrongSizeException {

        var numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
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
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator, true);

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
    void testCalibrateGeneralWithInlierNoise() throws IOException, LockedException, NotReadyException,
            WrongSizeException {

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
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
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
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator, true);

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
            NotReadyException, WrongSizeException {

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
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
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
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator, true);

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
            NotReadyException, WrongSizeException {

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
            final var qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {
                final var cnb = generateBodyC(randomizer);

                final StandardDeviationBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
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
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!bm.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bm.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator, false);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked((PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final int iteration) {
        checkLocked((PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final float progress) {
        checkLocked((PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator) calibrator);
        calibrateProgressChange++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
        calibrateNextIteration = 0;
        calibrateProgressChange = 0;
    }

    private static void checkLocked(final PROSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setThreshold(0.0));
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(1.0));
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(
                new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronX(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronY(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronZ(0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronX(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronY(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIronZ(null));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIron(
                0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIron(
                null, null, null));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIron((MagneticFluxDensityTriad) null));
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
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIron((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setInitialHardIron((Matrix) null));
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
            final Matrix hardIron, final Matrix mm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator, final boolean checkCovariance)
            throws WrongSizeException {

        final var estimatedHardIron = calibrator.getEstimatedHardIron();
        assertArrayEquals(hardIron.getBuffer(), estimatedHardIron, 0.0);

        final var estimatedHardIron2 = new double[3];
        assertTrue(calibrator.getEstimatedHardIron(estimatedHardIron2));
        assertArrayEquals(estimatedHardIron, estimatedHardIron2, 0.0);

        final var hardIron2 = new Matrix(3, 1);
        assertTrue(calibrator.getEstimatedHardIronAsMatrix(hardIron2));

        assertEquals(hardIron, hardIron2);

        assertEquals(hardIron.getElementAtIndex(0), calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(hardIron.getElementAtIndex(1), calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(hardIron.getElementAtIndex(2), calibrator.getEstimatedHardIronZ(), 0.0);

        final var bx1 = calibrator.getEstimatedHardIronXAsMagneticFluxDensity();
        assertEquals(calibrator.getEstimatedHardIronX(), bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        final var bx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        calibrator.getEstimatedHardIronXAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getEstimatedHardIronYAsMagneticFluxDensity();
        assertEquals(calibrator.getEstimatedHardIronY(), by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        final var by2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        calibrator.getEstimatedHardIronYAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getEstimatedHardIronZAsMagneticFluxDensity();
        assertEquals(calibrator.getEstimatedHardIronZ(), bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        final var bz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        calibrator.getEstimatedHardIronZAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);

        final var bTriad1 = calibrator.getEstimatedHardIronAsTriad();
        assertEquals(bTriad1.getValueX(), calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(bTriad1.getValueY(), calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(bTriad1.getValueZ(), calibrator.getEstimatedHardIronZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getEstimatedHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);

        assertEquals(mm.getElementAt(0, 0), calibrator.getEstimatedSx(), 0.0);
        assertEquals(mm.getElementAt(1, 1), calibrator.getEstimatedSy(), 0.0);
        assertEquals(mm.getElementAt(2, 2), calibrator.getEstimatedSz(), 0.0);
        assertEquals(mm.getElementAt(0, 1), calibrator.getEstimatedMxy(), 0.0);
        assertEquals(mm.getElementAt(0, 2), calibrator.getEstimatedMxz(), 0.0);
        assertEquals(mm.getElementAt(1, 0), calibrator.getEstimatedMyx(), 0.0);
        assertEquals(mm.getElementAt(1, 2), calibrator.getEstimatedMyz(), 0.0);
        assertEquals(mm.getElementAt(2, 0), calibrator.getEstimatedMzx(), 0.0);
        assertEquals(mm.getElementAt(2, 1), calibrator.getEstimatedMzy(), 0.0);

        if (checkCovariance) {
            assertCovariance(calibrator);
        }
    }

    private static void assertCovariance(final RobustKnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedHardIronXVariance());
        assertNotNull(calibrator.getEstimatedHardIronXStandardDeviation());
        final var stdBx1 = calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity();
        assertNotNull(stdBx1);
        final var stdBx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedHardIronYVariance());
        assertNotNull(calibrator.getEstimatedHardIronYStandardDeviation());
        final var stdBy1 = calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity();
        assertNotNull(stdBy1);
        final var stdBy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedHardIronZVariance());
        assertNotNull(calibrator.getEstimatedHardIronZStandardDeviation());
        final var stdBz1 = calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity();
        assertNotNull(stdBz1);
        final var stdBz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final var std1 = calibrator.getEstimatedHardIronStandardDeviation();
        assertEquals(std1.getValueX(), calibrator.getEstimatedHardIronXStandardDeviation(), 0.0);
        assertEquals(std1.getValueY(), calibrator.getEstimatedHardIronYStandardDeviation(), 0.0);
        assertEquals(std1.getValueZ(), calibrator.getEstimatedHardIronZStandardDeviation(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensityTriad();
        assertTrue(calibrator.getEstimatedHardIronStandardDeviation(std2));

        final var avgStd = (calibrator.getEstimatedHardIronXStandardDeviation()
                + calibrator.getEstimatedHardIronYStandardDeviation()
                + calibrator.getEstimatedHardIronZStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedHardIronStandardDeviationAverage(), 0.0);
        final var avg1 = calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedHardIronStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var norm1 = calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final var norm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(norm2);
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

    private static NEDMagneticFluxDensity getMagneticFluxDensityAtPosition(
            final NEDPosition position, final double year) throws IOException {
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        return wmmEstimator.estimate(position, year);
    }

    private static List<StandardDeviationBodyMagneticFluxDensity> generateMeasures(
            final double[] hardIron, final Matrix softIron, final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final UniformRandomizer randomizer,
            final NEDPosition position, final Date timestamp) {

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