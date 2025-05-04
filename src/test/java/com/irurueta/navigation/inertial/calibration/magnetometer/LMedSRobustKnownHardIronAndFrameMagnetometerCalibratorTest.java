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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
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
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class LMedSRobustKnownHardIronAndFrameMagnetometerCalibratorTest implements
        RobustKnownHardIronAndFrameMagnetometerCalibratorListener {

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
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 20;

    private static final double THRESHOLD = 10e-9;
    private static final double LARGE_THRESHOLD = 500e-9;

    private static final double OUTLIER_ERROR_FACTOR = 100.0;

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        var b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(this);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, this);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(true);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(true, 
                this);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                true);

        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                true, this);

        assertEquals(LMedSRobustKnownFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD, calibrator.getStopThreshold(),
                0.0);
        assertEquals(RobustEstimatorMethod.LMEDS, calibrator.getMethod());
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(new Matrix(3, 1), b);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, hardIronTriad1.getValueX(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueY(), 0.0);
        assertEquals(0.0, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                calibrator.getProgressDelta(), 0.0f);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE, calibrator.getConfidence(),
                0.0);
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS,
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
        assertEquals(RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());
        assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
    }

    @Test
    void testGetSetStopThreshold() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                calibrator.getStopThreshold(), 0.0);

        // set a new value
        calibrator.setStopThreshold(1.0);

        // check
        assertEquals(1.0, calibrator.getStopThreshold(), 0.0);
    }

    @Test
    void testGetSetHardIronX() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
    }

    @Test
    void testGetSetHardIronY() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronY = mb[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
    }

    @Test
    void testGetSetHardIronZ() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronZ = mb[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    void testGetSetHardIronXAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var mb = generateHardIron(randomizer);
        final var hardIronX = mb[0];
        final var hardIronY = mb[1];
        final var hardIronZ = mb[2];

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    void testSetHardIronCoordinates2() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = new MagneticFluxDensity(hardIron[0], MagneticFluxDensityUnit.TESLA);
        final var hardIronY = new MagneticFluxDensity(hardIron[1], MagneticFluxDensityUnit.TESLA);
        final var hardIronZ = new MagneticFluxDensity(hardIron[2], MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(hardIronX, calibrator.getHardIronXAsMagneticFluxDensity());
        assertEquals(hardIronY, calibrator.getHardIronYAsMagneticFluxDensity());
        assertEquals(hardIronZ, calibrator.getHardIronZAsMagneticFluxDensity());
    }

    @Test
    void testGetSetHardIronAsTriad() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        final var triad1 = calibrator.getHardIronAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());

        //set new values
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var triad2 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, hardIronX, hardIronY, hardIronZ);
        calibrator.setHardIron(triad2);

        // check
        final var triad3 = calibrator.getHardIronAsTriad();
        final var triad4 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetInitialSx() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);

        // set a new value
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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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

        // set a new value
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
    void testGetHardIronAsArray() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var result1 = new double[3];
        calibrator.getHardIron(result1);
        assertArrayEquals(new double[3], result1, 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bm = generateHardIron(randomizer);
        calibrator.setHardIron(bm);

        // check
        assertArrayEquals(calibrator.getHardIron(), bm, 0.0);
        final var result2 = new double[3];
        calibrator.getHardIron(result2);
        assertArrayEquals(result2, bm, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIron(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(new double[1]));
    }

    @Test
    void testGetHardIronAsMatrix() throws LockedException, WrongSizeException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(new Matrix(3, 1), calibrator.getHardIronMatrix());
        final var result1 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(result1);
        assertEquals(new Matrix(3, 1), result1);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bm = generateHardIron(randomizer);
        final var b = Matrix.newFromArray(bm);
        calibrator.setHardIron(b);

        // check
        assertEquals(b, calibrator.getHardIronMatrix());
        final var result2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(result2);
        assertEquals(result2, b);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIronMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIronMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(m3));
        final var m4 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(m4));
    }

    @Test
    void testGetSetInitialMm() throws WrongSizeException, LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var result1 = new Matrix(3, 3);
        calibrator.getInitialMm(result1);

        // set a new value
        final var mm = generateSoftIronGeneral();
        calibrator.setInitialMm(mm);

        // check
        assertEquals(mm, calibrator.getInitialMm());
        final var result2 = new Matrix(3, 3);
        calibrator.getInitialMm(result2);
        assertEquals(mm, result2);
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set a new value
        final var measurements = Collections.<StandardDeviationFrameBodyMagneticFluxDensity>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetListener() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set a new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException, IOException, InvalidSourceAndDestinationFrameTypeException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check initial value
        assertFalse(calibrator.isReady());

        // set not enough measurements
        calibrator.setMeasurements(Collections.emptyList());

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIronGeneral();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, softIron, 
                wmmEstimator, randomizer);

        calibrator.setMeasurements(measurements);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetMagneticModel() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set a new value
        final var magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    void testIsSetLinearCalibratorUsed() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isLinearCalibratorUsed());

        // set a new value
        calibrator.setLinearCalibratorUsed(false);

        // check
        assertFalse(calibrator.isLinearCalibratorUsed());
    }

    @Test
    void testIsSetPreliminarySolutionRefined() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isPreliminarySolutionRefined());

        // set a new value
        calibrator.setPreliminarySolutionRefined(true);

        // check
        assertTrue(calibrator.isPreliminarySolutionRefined());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set a new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    void testIsSetCovarianceKept() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set a new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    void testGetSetQualityScores() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set a new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    void testGetSetPreliminarySubsetSize() throws LockedException {
        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getPreliminarySubsetSize());

        // set a new value
        calibrator.setPreliminarySubsetSize(4);

        // check
        assertEquals(4, calibrator.getPreliminarySubsetSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.setPreliminarySubsetSize(2));
    }

    @Test
    void testCalibrateGeneralNoNoiseInlier() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
        for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

            final StandardDeviationFrameBodyMagneticFluxDensity b;
            if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                // outlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, noiseRandomizer, position);
            } else {
                // inlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, null, position);
            }
            measurements.add(b);
        }

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                false, this);
        calibrator.setHardIron(hardIron);
        calibrator.setStopThreshold(THRESHOLD);

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

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkGeneralCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() > 0.0);
        assertNotEquals(0.0, calibrator.getEstimatedChiSq());
    }

    @Test
    void testCalibrateCommonAxisNoNoiseInlier() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
        for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

            final StandardDeviationFrameBodyMagneticFluxDensity b;
            if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                // outlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, noiseRandomizer, position);
            } else {
                // inlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, null, position);
            }
            measurements.add(b);
        }

        final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                true, this);
        calibrator.setHardIron(hardIron);
        calibrator.setStopThreshold(THRESHOLD);

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

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
        checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedMse() > 0.0);
        assertNotEquals(0.0, calibrator.getEstimatedChiSq());
    }

    @Test
    void testCalibrateGeneralWithInlierNoise() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var inlierNoiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);
            final var outlierNoiseRandomizer = new GaussianRandomizer(0.0, 
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, outlierNoiseRandomizer, 
                            position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, inlierNoiseRandomizer,
                            position);
                }
                measurements.add(b);
            }

            final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                    false, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateCommonAxisWithInlierNoise() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var inlierNoiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);
            final var outlierNoiseRandomizer = new GaussianRandomizer(0.0, 
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, outlierNoiseRandomizer,
                            position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, inlierNoiseRandomizer,
                            position);
                }
                measurements.add(b);
            }

            final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements,
                    true, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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
    void testCalibrateGeneralNoRefinement() throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, noiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, null,
                            position);
                }
                measurements.add(b);
            }

            final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                    false, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setResultRefined(false);
            calibrator.setPreliminarySolutionRefined(false);

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

            assertNull(calibrator.getEstimatedCovariance());
            assertEquals(0.0, calibrator.getEstimatedMse(), 0.0);
            assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateGeneralNonLinearWithInitialValue() throws IOException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var measurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
            for (var i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, noiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator, randomizer, null,
                            position);
                }
                measurements.add(b);
            }

            final var calibrator = new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, 
                    false, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setInitialMm(mm);
            calibrator.setLinearCalibratorUsed(false);
            calibrator.setPreliminarySolutionRefined(true);

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
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        calibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, final int iteration) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        calibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator, final float progress) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        calibrateProgressChange++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
        calibrateNextIteration = 0;
        calibrateProgressChange = 0;
    }

    private void checkLocked(final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setStopThreshold(0.0));
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
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, () -> calibrator.setMagneticModel(null));
        assertThrows(LockedException.class, () -> calibrator.setLinearCalibratorUsed(false));
        assertThrows(LockedException.class, () -> calibrator.setPreliminarySolutionRefined(false));
        assertThrows(LockedException.class, () -> calibrator.setProgressDelta(0.5f));
        assertThrows(LockedException.class, () -> calibrator.setConfidence(0.5));
        assertThrows(LockedException.class, () -> calibrator.setMaxIterations(100));
        assertThrows(LockedException.class, () -> calibrator.setResultRefined(true));
        assertThrows(LockedException.class, () -> calibrator.setCovarianceKept(true));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix mm, final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {

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

    private static List<StandardDeviationFrameBodyMagneticFluxDensity>
    generateMeasurementsMultipleOrientationsWithSamePosition(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer) throws InvalidSourceAndDestinationFrameTypeException {
        final var position = createPosition(randomizer);
        final var result = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
        for (var i = 0; i < RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS; i++) {
            result.add(generateMeasureAtPosition(hardIron, softIron, wmmEstimator, randomizer, null,
                    position));
        }
        return result;
    }

    private static StandardDeviationFrameBodyMagneticFluxDensity generateMeasureAtPosition(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer, final GaussianRandomizer noiseRandomizer, final NEDPosition position)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var cnb = generateBodyC(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator, randomizer, noiseRandomizer, position, cnb);
    }

    private static StandardDeviationFrameBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer, final GaussianRandomizer noiseRandomizer,
            final NEDPosition position, final CoordinateTransformation cnb)
            throws InvalidSourceAndDestinationFrameTypeException {

        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        final var cbn = cnb.inverseAndReturnNew();
        final var frame = new NEDFrame(position, cbn);

        final var std = noiseRandomizer != null ? noiseRandomizer.getStandardDeviation() : MAGNETOMETER_NOISE_STD;
        return new StandardDeviationFrameBodyMagneticFluxDensity(measuredMagnetic, frame, timestamp, std);
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
