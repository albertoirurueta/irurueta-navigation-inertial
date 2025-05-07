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

class KnownMagneticFluxDensityNormMagnetometerCalibratorTest implements
        KnownMagneticFluxDensityNormMagnetometerCalibratorListener {

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

    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

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

    @Test
    void testConstructor1() throws WrongSizeException {
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        var b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        var b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(true);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                new double[1]));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix1 = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(hardIronMatrix1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix2, hardIronMatrix1);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m2));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix1 = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(hardIronMatrix1, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix2, hardIronMatrix1);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m1,
                mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m2,
                mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                hardIronMatrix1, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                hardIronMatrix1, m4));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var randomizer = new UniformRandomizer();
        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, new double[1]));
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIron1, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, new double[1], this));
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, new double[1]));
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, hardIron1, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, new double[1], this));
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m2));
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIronMatrix, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m2, this));
    }

    @Test
    void testConstructor17() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m2));
    }

    @Test
    void testConstructor18() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, 
                true, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m2, this));
    }

    @Test
    void testConstructor19() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIronMatrix, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m1, mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, hardIronMatrix, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, hardIronMatrix, m4));
    }

    @Test
    void testConstructor20() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, hardIronMatrix, mm, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m1, mm, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, hardIronMatrix, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, hardIronMatrix, m4, this));
    }

    @Test
    void testConstructor21() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, true,
                hardIronMatrix, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m1, mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, hardIronMatrix, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, hardIronMatrix, m4));
    }

    @Test
    void testConstructor22() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();

        final var measurements = Collections.<StandardDeviationBodyMagneticFluxDensity>emptyList();

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(measurements, true,
                hardIronMatrix, mm, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m1, mm, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, hardIronMatrix, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                measurements, true, hardIronMatrix, m4, this));
    }

    @Test
    void testConstructor23() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
    }

    @Test
    void testConstructor24() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
    }

    @Test
    void testConstructor26() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, true);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
    }

    @Test
    void testConstructor27() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, new double[1]));
    }

    @Test
    void testConstructor28() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix1 = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIronMatrix1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix2, hardIronMatrix1);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(m2));
    }

    @Test
    void testConstructor29() throws WrongSizeException, IOException {
        final var randomizer = new UniformRandomizer();
        final var position = createPosition(randomizer);
        final var calendar = new GregorianCalendar();
        calendar.setTime(new Date(createTimestamp(randomizer)));
        final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
        final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();
        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix1 = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIronMatrix1, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix2, hardIronMatrix1);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, m1, mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIronMatrix1, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, hardIronMatrix1, m4));
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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertEquals(groundTruthMagneticFluxDensityNorm, calibrator.getGroundTruthMagneticFluxDensityNorm(), 
                0.0);
        final var b3 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertEquals(groundTruthMagneticFluxDensityNorm, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final var b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b4));
        assertEquals(b3, b4);
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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, this);

        // check default values
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
        assertArrayEquals(new double[3], calibrator.getInitialHardIron(), 0.0);
        final var hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(new double[3], hardIron, 0.0);
        assertEquals(new Matrix(3, 1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(new Matrix(3, 1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(0.0, bTriad1.getValueX(), 0.0);
        assertEquals(0.0, bTriad1.getValueY(), 0.0);
        assertEquals(0.0, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(hardIronMatrix, Matrix.newFromArray(hardIron1));
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, new double[1]));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIron1, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(Matrix.newFromArray(hardIron1), hardIronMatrix);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, new double[1], this));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron1);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(hardIronMatrix, Matrix.newFromArray(hardIron1));
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, new double[1]));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIron1, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix);
        assertEquals(hardIronMatrix, Matrix.newFromArray(hardIron1));
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, new double[1], this));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m2));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m2, this));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m2));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(0.0, calibrator.getInitialSx(), 0.0);
        assertEquals(0.0, calibrator.getInitialSy(), 0.0);
        assertEquals(0.0, calibrator.getInitialSz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxy(), 0.0);
        assertEquals(0.0, calibrator.getInitialMxz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMyz(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzx(), 0.0);
        assertEquals(0.0, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(new Matrix(3, 3), mm);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m2, this));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m1, mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, m4));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, mm, this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m1, mm, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, m3, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, hardIronMatrix, m4, this));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, mm);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, m4));
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

        final var hardIron1 = generateHardIron(randomizer);
        final var hardIronMatrix = Matrix.newFromArray(hardIron1);
        final var hardIronX = hardIron1[0];
        final var hardIronY = hardIron1[1];
        final var hardIronZ = hardIron1[2];

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

        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, mm, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getInitialHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getInitialHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getInitialHardIronZ(), 0.0);
        assertEquals(sx, calibrator.getInitialSx(), 0.0);
        assertEquals(sy, calibrator.getInitialSy(), 0.0);
        assertEquals(sz, calibrator.getInitialSz(), 0.0);
        assertEquals(mxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(mxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(myx, calibrator.getInitialMyx(), 0.0);
        assertEquals(myz, calibrator.getInitialMyz(), 0.0);
        assertEquals(mzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(mzy, calibrator.getInitialMzy(), 0.0);
        assertArrayEquals(hardIron1, calibrator.getInitialHardIron(), 0.0);
        final var hardIron2 = new double[3];
        calibrator.getInitialHardIron(hardIron2);
        assertArrayEquals(hardIron2, hardIron1, 0.0);
        assertEquals(Matrix.newFromArray(hardIron1), calibrator.getInitialHardIronAsMatrix());
        final var hardIronMatrix2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(hardIronMatrix2);
        assertEquals(hardIronMatrix, hardIronMatrix2);
        MagneticFluxDensity b1 = calibrator.getInitialHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getInitialHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getInitialHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getInitialHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var bTriad1 = calibrator.getInitialHardIronAsTriad();
        assertEquals(hardIronX, bTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, bTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, bTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bTriad1.getUnit());
        final var bTriad2 = new MagneticFluxDensityTriad();
        calibrator.getInitialHardIronAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);
        assertEquals(mm, calibrator.getInitialMm());
        final var mm2 = new Matrix(3, 3);
        calibrator.getInitialMm(mm2);
        assertEquals(mm, mm2);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedHardIron());
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
        assertEquals(0.0, calibrator.getEstimatedChiSq(), 0.0);
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
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m1, mm, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, m2, mm, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, m3,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                groundTruthMagneticFluxDensityNorm, measurements, true, hardIronMatrix, m4,
                this));
    }

    @Test
    void testGetSetGroundTruthMagneticFluxDensityNorm1() throws LockedException {
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertNull(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity());
        final var norm1 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertFalse(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(norm1));

        // set new value
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        final var def = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);

        // check default value
        assertEquals(def, calibrator.getInitialHardIronXAsMagneticFluxDensity());
        assertEquals(def, calibrator.getInitialHardIronYAsMagneticFluxDensity());
        assertEquals(def, calibrator.getInitialHardIronZAsMagneticFluxDensity());

        // set new value
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        assertEquals(calibrator.getInitialHardIronAsMatrix(), hardIron2);
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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check initial value
        assertEquals(new Matrix(3, 3), calibrator.getInitialMm());
        final var mm1 = new Matrix(3, 3);
        calibrator.getInitialMm(mm1);
        assertEquals(new Matrix(3, 3), mm1);

        // set new value
        final var mm2 = generateSoftIronGeneral();
        calibrator.setInitialMm(mm2);

        // check
        assertEquals(calibrator.getInitialMm(), mm2);
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
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set a new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set a new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetMinimumRequiredMeasurements() throws LockedException {
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator();

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
        final var measurements2 = generateMeasures(hardIron, softIron, 
                KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL, wmmEstimator, randomizer, 
                null, position, timestamp);
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
    void testCalibrateForGeneralCaseWithMinimumMeasuresAndNoNoise() throws IOException, LockedException, 
            NotReadyException, WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = generateMeasures(hardIron.getBuffer(), mm, 
                    KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL, wmmEstimator, 
                    randomizer, null, position, timestamp);
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();


            final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, false, hardIron, mm, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(0, calibrateStart);
            assertEquals(0, calibrateEnd);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException ignore) {
                continue;
            }

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(1, calibrateStart);
            assertEquals(1, calibrateEnd);

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

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
    void testCalibrateForGeneralCaseWithLargeNumberOfMeasurementsAndNoise() throws IOException, LockedException,
            CalibrationException, NotReadyException, WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = generateMeasures(hardIron.getBuffer(), mm, LARGE_MEASUREMENT_NUMBER, wmmEstimator,
                    randomizer, noiseRandomizer, position, timestamp);
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, false, hardIron, mm, this);

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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, 5.0 * VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, 5.0 * VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

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
    void testCalibrateForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws IOException, LockedException,
            NotReadyException, WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = generateMeasures(hardIron.getBuffer(), mm,
                    KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL, wmmEstimator,
                    randomizer, null, position, timestamp);
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, mm, this);

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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() >= 0.0);
            if (calibrator.getEstimatedMse() > 0.0) {
                assertNotEquals(0.0, calibrator.getEstimatedChiSq());
            }

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateForCommonAxisCaseWithLargeNumberOfMeasurementsAndNoise() throws IOException, LockedException,
            CalibrationException, NotReadyException, WrongSizeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var position = createPosition(randomizer);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var measurements = generateMeasures(hardIron.getBuffer(), mm, LARGE_MEASUREMENT_NUMBER, wmmEstimator,
                    randomizer, noiseRandomizer, position, timestamp);
            final var calendar = new GregorianCalendar();
            calendar.setTime(timestamp);
            final var year = WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
            final var groundTruthMagneticFluxDensityNorm = getMagneticFluxDensityAtPosition(position, year).getNorm();

            final var calibrator = new KnownMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, true, hardIron, mm, this);

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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, 5.0 * VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, 5.0 * VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(0.0, calibrator.getEstimatedChiSq());

            numValid++;

            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(1.0));
        assertThrows(LockedException.class, () -> calibrator.setGroundTruthMagneticFluxDensityNorm(
                new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA)));
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix hardIron, final Matrix mm,
            final KnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) throws WrongSizeException {

        final var estimatedHardIron = calibrator.getEstimatedHardIron();
        assertArrayEquals(estimatedHardIron, hardIron.getBuffer(), 0.0);

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
        assertEquals(bx1.getValue().doubleValue(), calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        final var bx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        calibrator.getEstimatedHardIronXAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        final var by1 = calibrator.getEstimatedHardIronYAsMagneticFluxDensity();
        assertEquals(by1.getValue().doubleValue(), calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        final var by2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        calibrator.getEstimatedHardIronYAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        final var bz1 = calibrator.getEstimatedHardIronZAsMagneticFluxDensity();
        assertEquals(bz1.getValue().doubleValue(), calibrator.getEstimatedHardIronZ(), 0.0);
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

        assertCovariance(calibrator);

        assertNotNull(calibrator.getGroundTruthMagneticFluxDensityNorm());
        final var b1 = calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
        assertNotNull(b1);
        assertEquals(b1.getValue(), calibrator.getGroundTruthMagneticFluxDensityNorm());
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertTrue(calibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(b2));
        assertEquals(b1, b2);
    }

    private static void assertCovariance(final KnownMagneticFluxDensityNormMagnetometerCalibrator calibrator) {
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
            final GaussianRandomizer noiseRandomizer, final NEDPosition position, final Date timestamp) {

        final var result = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
        for (var i = 0; i < numberOfMeasurements; i++) {
            final var cnb = generateBodyC(randomizer);
            result.add(generateMeasure(hardIron, softIron, wmmEstimator, noiseRandomizer, position, timestamp, cnb));
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