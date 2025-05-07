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
import com.irurueta.navigation.inertial.calibration.FrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
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

class KnownFrameMagnetometerLinearLeastSquaresCalibratorTest implements 
        KnownFrameMagnetometerLinearLeastSquaresCalibratorListener {

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

    private static final int SMALL_MEASUREMENT_NUMBER = 16;
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
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, true,
                this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor9() throws WrongSizeException {
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(model);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), model);
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
    }

    @Test
    void testConstructor10() throws WrongSizeException {
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(model, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), model);
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
    }

    @Test
    void testConstructor11() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, model);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor12() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, model, 
                this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor13() throws WrongSizeException {
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(true, model);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor14() throws WrongSizeException {
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(true, model, 
                this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor15() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, model);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testConstructor16() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var model = new WorldMagneticModel();
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, true,
                model, this);

        // check default values
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(model, calibrator.getMagneticModel());
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
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    void testIsSetCommonAxisUsed() throws LockedException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testIsReady() throws LockedException, IOException, InvalidSourceAndDestinationFrameTypeException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

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
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        calibrator.setMeasurements(measurements);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    void testGetSetMagneticModel() throws LockedException {
        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final var model = new WorldMagneticModel();
        calibrator.setMagneticModel(model);

        // check
        assertSame(model, calibrator.getMagneticModel());
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
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

        final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
        final var estimatedMm = calibrator.getEstimatedMm();

        assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), ABSOLUTE_ERROR);
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var model = wmmEstimator.getModel();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                false, model, this);

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

        assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), ABSOLUTE_ERROR);
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }


    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                    LARGE_MEASUREMENT_NUMBER, wmmEstimator, randomizer, noiseRandomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), LARGE_ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseSmallNumberOfMeasurements() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                    SMALL_MEASUREMENT_NUMBER, wmmEstimator, randomizer, noiseRandomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), VERY_LARGE_ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var measurements = generateMeasurementsMultiplePositionsWithSameOrientation(hardIron, mm,
                    wmmEstimator, randomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(estimatedHardIron.getBuffer(), hardIron, ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, true,
                this);

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

        assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), ABSOLUTE_ERROR);
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var model = wmmEstimator.getModel();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, true,
                model, this);

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

        assertArrayEquals(hardIron, estimatedHardIron.getBuffer(), ABSOLUTE_ERROR);
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm,
                    LARGE_MEASUREMENT_NUMBER, wmmEstimator, randomizer, noiseRandomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(estimatedHardIron.getBuffer(), hardIron, LARGE_ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseSmallNumberOfMeasurements() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

            final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm, 
                    SMALL_MEASUREMENT_NUMBER, wmmEstimator, randomizer, noiseRandomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(estimatedHardIron.getBuffer(), hardIron, VERY_LARGE_ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            WrongSizeException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var measurements = generateMeasurementsMultiplePositionsWithSameOrientation(hardIron, mm, 
                    wmmEstimator, randomizer);

            final var calibrator = new KnownFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
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

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            final var hardIronMatrix = Matrix.newFromArray(hardIron);
            if (!hardIronMatrix.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertArrayEquals(estimatedHardIron.getBuffer(), hardIron, ABSOLUTE_ERROR);
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, () -> calibrator.setMagneticModel(null));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix hardIron, final Matrix mm, final KnownFrameMagnetometerLinearLeastSquaresCalibrator calibrator)
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

        final var hardIronTriad1 = calibrator.getEstimatedHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(hardIronTriad1.getValueY(), calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(hardIronTriad1.getValueZ(), calibrator.getEstimatedHardIronZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getEstimatedHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);

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

    private static List<FrameBodyMagneticFluxDensity> generateMeasurementsMultipleOrientationsWithSamePosition(
            final double[] hardIron, final Matrix softIron, final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final UniformRandomizer randomizer,
            final GaussianRandomizer noiseRandomizer) throws InvalidSourceAndDestinationFrameTypeException {
        final var position = createPosition(randomizer);
        final var result = new ArrayList<FrameBodyMagneticFluxDensity>();
        for (var i = 0; i < numberOfMeasurements; i++) {
            result.add(generateMeasureAtPosition(hardIron, softIron, wmmEstimator, randomizer, noiseRandomizer,
                    position));
        }
        return result;
    }

    private static List<FrameBodyMagneticFluxDensity> generateMeasurementsMultiplePositionsWithSameOrientation(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer) throws InvalidSourceAndDestinationFrameTypeException {
        final var cnb = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
        final var result = new ArrayList<FrameBodyMagneticFluxDensity>();
        for (var i = 0; i < KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
            result.add(generateMeasureAtOrientation(hardIron, softIron, wmmEstimator, randomizer, cnb));
        }
        return result;
    }

    private static FrameBodyMagneticFluxDensity generateMeasureAtOrientation(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer, final CoordinateTransformation cnb)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var position = createPosition(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator, randomizer, null, position, cnb);
    }

    private static FrameBodyMagneticFluxDensity generateMeasureAtPosition(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer, final GaussianRandomizer noiseRandomizer, final NEDPosition position)
            throws InvalidSourceAndDestinationFrameTypeException {
        final var cnb = generateBodyC(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator, randomizer, noiseRandomizer, position, cnb);
    }

    private static FrameBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer, final GaussianRandomizer noiseRandomizer, final NEDPosition position,
            final CoordinateTransformation cnb) throws InvalidSourceAndDestinationFrameTypeException {

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

        return new FrameBodyMagneticFluxDensity(measuredMagnetic, frame, timestamp);
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
