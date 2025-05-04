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

class KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibratorTest implements
        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibratorListener {

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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, 
                this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel, this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel, this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
        final var magneticModel = new WorldMagneticModel();
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel, this);

        // check default values
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(new double[3], calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(new double[3], hardIron1, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(new Matrix(3, 1), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor17() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor18() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor19() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, 
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor20() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor21() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor22() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor23() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor24() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor25() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel,
                hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor26() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, 
                hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor27() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor28() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel, hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor29() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor30() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel, hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor31() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor32() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel, hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
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
    void testConstructor33() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(new double[1]));
    }

    @Test
    void testConstructor34() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(hardIron, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(new double[1], this));
    }

    @Test
    void testConstructor35() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, new double[1]));
    }

    @Test
    void testConstructor36() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, hardIron, 
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, new double[1],
                        this));
    }

    @Test
    void testConstructor37() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        new double[1]));
    }

    @Test
    void testConstructor38() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        new double[1], this));
    }

    @Test
    void testConstructor39() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                true, hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, new double[1]));
    }

    @Test
    void testConstructor40() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                true, hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, new double[1], this));
    }

    @Test
    void testConstructor41() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel,
                hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, new double[1]));
    }

    @Test
    void testConstructor42() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel,
                hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, new double[1],
                        this));
    }

    @Test
    void testConstructor43() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                magneticModel, hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        new double[1]));
    }

    @Test
    void testConstructor44() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                magneticModel, hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        new double[1], this));
    }

    @Test
    void testConstructor45() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                magneticModel, hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, new double[1]));
    }

    @Test
    void testConstructor46() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                magneticModel, hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, new double[1], this));
    }

    @Test
    void testConstructor47() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                true, magneticModel, hardIron);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, new double[1]));
    }

    @Test
    void testConstructor48() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();

        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                true, magneticModel, hardIron, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(Matrix.newFromArray(hardIron), hardIron2);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, new double[1], this));
    }

    @Test
    void testConstructor49() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(m2));
    }

    @Test
    void testConstructor50() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(hardIronMatrix,
                this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(m2, this));
    }

    @Test
    void testConstructor51() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, m2));
    }

    @Test
    void testConstructor52() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, m1,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, m2,
                        this));
    }

    @Test
    void testConstructor53() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, m2));
    }

    @Test
    void testConstructor54() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, m1,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, m2,
                        this));
    }

    @Test
    void testConstructor55() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                true, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, 
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                        true, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, m2));
    }

    @Test
    void testConstructor56() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, m2, this));
    }

    @Test
    void testConstructor57() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, 
                hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, m2));
    }

    @Test
    void testConstructor58() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, 
                hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, m1,
                        this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(magneticModel, m2,
                        this));
    }

    @Test
    void testConstructor59() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        m2));
    }

    @Test
    void testConstructor60() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                magneticModel, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, magneticModel,
                        m2, this));
    }

    @Test
    void testConstructor61() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, m2));
    }

    @Test
    void testConstructor62() throws WrongSizeException {
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true, 
                magneticModel, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertNull(calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(true,
                        magneticModel, m2, this));
    }

    @Test
    void testConstructor63() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel, hardIronMatrix);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY, 
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, m2));
    }

    @Test
    void testConstructor64() throws WrongSizeException {
        final var measurements = Collections.<FrameBodyMagneticFluxDensity>emptyList();
        final var magneticModel = new WorldMagneticModel();
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];
        final var hardIronY = hardIron[1];
        final var hardIronZ = hardIron[2];

        final var hardIronMatrix = Matrix.newFromArray(hardIron);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, magneticModel, hardIronMatrix, this);

        // check default values
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
        assertSame(measurements, calibrator.getMeasurements());
        assertEquals(MagnetometerCalibratorMeasurementType.FRAME_BODY_MAGNETIC_FLUX_DENSITY,
                calibrator.getMeasurementType());
        assertFalse(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(this, calibrator.getListener());
        assertEquals(KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurements());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(magneticModel, calibrator.getMagneticModel());
        assertArrayEquals(hardIron, calibrator.getHardIron(), 0.0);
        final var hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final var hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final var hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(hardIronX, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final var b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(hardIronY, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(hardIronZ, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final var hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronX, hardIronTriad1.getValueX(), 0.0);
        assertEquals(hardIronY, hardIronTriad1.getValueY(), 0.0);
        assertEquals(hardIronZ, hardIronTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, hardIronTriad1.getUnit());
        final var hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
        assertNull(calibrator.getEstimatedMm());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, m1, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class,
                () -> new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements,
                        true, magneticModel, m2, this));
    }

    @Test
    void testGetSetHardIronX() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronX = hardIron[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(hardIronX, calibrator.getHardIronX(), 0.0);
    }

    @Test
    void testGetSetHardIronY() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronY = hardIron[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(hardIronY, calibrator.getHardIronY(), 0.0);
    }

    @Test
    void testGetSetHardIronZ() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getHardIronZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var hardIron = generateHardIron(randomizer);
        final var hardIronZ = hardIron[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(hardIronZ, calibrator.getHardIronZ(), 0.0);
    }

    @Test
    void testGetSetHardIronXAsMagneticFluxDensity() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final var b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
    void testGetSetMeasurements() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(this, calibrator.getListener());
    }

    @Test
    void testGetSetMagneticModel() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final var magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(magneticModel, calibrator.getMagneticModel());
    }

    @Test
    void testGetSetHardIron() throws LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final var hardIron1 = calibrator.getHardIron();
        final var hardIron2 = new double[3];
        calibrator.getHardIron(hardIron2);

        assertArrayEquals(new double[3], hardIron1, 0.0);
        assertArrayEquals(hardIron1, hardIron2, 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var hardIron3 = generateHardIron(randomizer);

        calibrator.setHardIron(hardIron3);

        // check
        final var hardIron4 = calibrator.getHardIron();
        final var hardIron5 = new double[3];
        calibrator.getHardIron(hardIron5);

        assertArrayEquals(hardIron3, hardIron4, 0.0);
        assertArrayEquals(hardIron3, hardIron5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> calibrator.getHardIron(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> calibrator.setHardIron(new double[1]));
    }

    @Test
    void testGetSetHardIronMatrix() throws WrongSizeException, LockedException {
        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final var hardIron1 = calibrator.getHardIronMatrix();
        final var hardIron2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron2);

        assertEquals(new Matrix(3, 1), hardIron1);
        assertEquals(hardIron1, hardIron2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var hardIron3 = Matrix.newFromArray(generateHardIron(randomizer));

        calibrator.setHardIron(hardIron3);

        // check
        final var hardIron4 = calibrator.getHardIronMatrix();
        final var hardIron5 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron5);

        assertEquals(hardIron3, hardIron4);
        assertEquals(hardIron3, hardIron5);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel() 
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm, 
                KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, 
                randomizer, null);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                false, hardIron, this);

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

        final var estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var model = wmmEstimator.getModel();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm, 
                KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, 
                randomizer, null);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                false, model, hardIron, this);

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

        final var estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

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

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    false, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseSmallNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

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

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    false, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise() throws IOException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var measurements = generateMeasurementsMultiplePositionsWithSameOrientation(hardIron, mm, 
                    wmmEstimator, randomizer);

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    false, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm, 
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, hardIron, this);

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

        final var estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndMagneticModel() 
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException {

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var model = wmmEstimator.getModel();

        final var hardIron = generateHardIron(randomizer);
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var measurements = generateMeasurementsMultipleOrientationsWithSamePosition(hardIron, mm, 
                KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS, wmmEstimator, randomizer,
                null);

        final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                true, model, hardIron, this);

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

        final var estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

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

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    true, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm,
                    VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseSmallNumberOfMeasurements() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

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

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    true, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise() throws IOException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

            final var hardIron = generateHardIron(randomizer);
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var measurements = generateMeasurementsMultiplePositionsWithSameOrientation(hardIron, mm, 
                    wmmEstimator, randomizer);

            final var calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(measurements, 
                    true, hardIron, this);

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

            final var estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        calibrateEnd++;
    }

    private void reset() {
        calibrateStart = 0;
        calibrateEnd = 0;
    }

    private void checkLocked(final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
        assertThrows(LockedException.class, () -> calibrator.setMeasurements(null));
        assertThrows(LockedException.class, () -> calibrator.setCommonAxisUsed(true));
        assertThrows(LockedException.class, () -> calibrator.setListener(this));
        assertThrows(LockedException.class, () -> calibrator.setMagneticModel(null));
        assertThrows(LockedException.class, () -> calibrator.setHardIron((double[]) null));
        assertThrows(LockedException.class, () -> calibrator.setHardIron((Matrix) null));
        assertThrows(LockedException.class, calibrator::calibrate);
    }

    private static void assertEstimatedResult(
            final Matrix mm, final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {

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
