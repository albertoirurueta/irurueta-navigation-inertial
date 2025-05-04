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
package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class MagnetometerMeasurementGeneratorTest implements MagnetometerMeasurementsGeneratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

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

    private static final double SMALL_MAGNETOMETER_NOISE_STD = 1e-12;
    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private int initializationStarted;
    private int initializationCompleted;
    private int error;
    private int staticIntervalDetected;
    private int dynamicIntervalDetected;
    private int staticIntervalSkipped;
    private int mDynamicIntervalSkipped;
    private int mGeneratedMeasurement;
    private int reset;

    private final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();

    @Test
    void testConstructor1() {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default values
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.getMaxDynamicSamples());
        assertNull(generator.getListener());
        assertEquals(0, generator.getProcessedStaticSamples());
        assertEquals(0, generator.getProcessedDynamicSamples());
        assertFalse(generator.isStaticIntervalSkipped());
        assertFalse(generator.isDynamicIntervalSkipped());
        assertFalse(generator.isRunning());

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, generator.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, generator.getThresholdFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR, 
                generator.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, 
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(), generator.getBaseNoiseLevelAbsoluteThreshold(), 
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, errorThreshold1.getUnit());
        final var errorThreshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final var baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, baseNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final var baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(0.0, generator.getThreshold(), 0.0);
        final var threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(0.0, threshold1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Test
    void testConstructor2() {
        final var generator = new MagnetometerMeasurementsGenerator(this);

        // check default values
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.getMaxDynamicSamples());
        assertSame(this, generator.getListener());
        assertEquals(0, generator.getProcessedStaticSamples());
        assertEquals(0, generator.getProcessedDynamicSamples());
        assertFalse(generator.isStaticIntervalSkipped());
        assertFalse(generator.isDynamicIntervalSkipped());
        assertFalse(generator.isRunning());

        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, generator.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, generator.getThresholdFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                generator.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(), generator.getBaseNoiseLevelAbsoluteThreshold(), 
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, errorThreshold1.getUnit());
        final var errorThreshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final var baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, baseNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final var baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(0.0, generator.getThreshold(), 0.0);
        final var threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(0.0, threshold1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Test
    void testGetSetMinStaticSamples() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, generator.getMinStaticSamples());

        // set new value
        generator.setMinStaticSamples(10);

        // check
        assertEquals(10, generator.getMinStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setMinStaticSamples(1));
    }

    @Test
    void testGetSetMaxDynamicSamples() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, generator.getMaxDynamicSamples());

        // set new value
        generator.setMaxDynamicSamples(10);

        // check
        assertEquals(10, generator.getMaxDynamicSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setMaxDynamicSamples(1));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(this, generator.getListener());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, generator.getWindowSize());

        // set new value
        generator.setWindowSize(3);

        // check
        assertEquals(3, generator.getWindowSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setWindowSize(1));
        assertThrows(IllegalArgumentException.class, () -> generator.setWindowSize(2));
    }

    @Test
    void testGetSetInitialStaticSamples() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, generator.getInitialStaticSamples());

        // set new value
        generator.setInitialStaticSamples(2);

        // check
        assertEquals(2, generator.getInitialStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setInitialStaticSamples(1));
    }

    @Test
    void testGetSetThresholdFactor() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, generator.getThresholdFactor(), 0.0);

        // set new value
        generator.setThresholdFactor(1.0);

        // check
        assertEquals(1.0, generator.getThresholdFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setThresholdFactor(0.0));
    }

    @Test
    void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR, 
                generator.getInstantaneousNoiseLevelFactor(), 0.0);

        // set new value
        generator.setInstantaneousNoiseLevelFactor(1.0);

        // check
        assertEquals(1.0, generator.getInstantaneousNoiseLevelFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setInstantaneousNoiseLevelFactor(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD, 
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // set new value
        generator.setBaseNoiseLevelAbsoluteThreshold(1.0);

        // check
        assertEquals(1.0, generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setBaseNoiseLevelAbsoluteThreshold(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {
        final var generator = new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final var a1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());

        // set new value
        final var a2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        generator.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final var a3 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final var a4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    void testProcessCalibrateAndResetWithSmallNoiseAndCommonAxis() throws WrongSizeException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(0.0, SMALL_MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            var cnb = generateBodyC(randomizer);

            var nedC = cnb.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(measurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, reset);

            final var generator = new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer);

                assertEquals(i + 1, staticIntervalDetected);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, 
                        false);

                assertEquals(i + 1, dynamicIntervalDetected);
                assertEquals(i + 1, measurements.size());
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, measurements, 
                    true);
            calibrator.setTime(timestamp);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException ignore) {
                continue;
            }

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessCalibrateAndResetWithSmallNoiseAndGeneralCase() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            var cnb = generateBodyC(randomizer);

            var nedC = cnb.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(measurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, reset);

            final var generator = new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer);

                assertEquals(i + 1, staticIntervalDetected);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, 
                        true);

                assertEquals(i + 1, dynamicIntervalDetected);
                assertEquals(i + 1, measurements.size());
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, measurements,
                    false);
            calibrator.setTime(timestamp);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessCalibrateAndResetWithNoiseAndCommonAxis() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            var cnb = generateBodyC(randomizer);

            var nedC = cnb.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(measurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, reset);

            final var generator = new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer);

                assertEquals(i + 1, staticIntervalDetected);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, 
                        true);

                assertEquals(i + 1, dynamicIntervalDetected);
                assertEquals(i + 1, measurements.size());
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, measurements, 
                    true);
            calibrator.setTime(timestamp);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessCalibrateAndResetWithNoiseAndGeneralCase() throws WrongSizeException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            var cnb = generateBodyC(randomizer);

            var nedC = cnb.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(measurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, reset);

            final var generator = new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer);

                assertEquals(i + 1, staticIntervalDetected);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, 
                        true);

                assertEquals(i + 1, dynamicIntervalDetected);
                assertEquals(i + 1, measurements.size());
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, measurements, 
                    false);
            calibrator.setTime(timestamp);

            try {
                calibrator.calibrate();
            } catch (CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessSmallNoiseWithRotationAndPositionChange() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            var cnb = generateBodyC(randomizer);

            var nedC = cnb.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(measurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, reset);

            final var generator = new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer);

                assertEquals(i + 1, staticIntervalDetected);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, 
                        true);

                assertEquals(i + 1, dynamicIntervalDetected);
                assertEquals(i + 1, measurements.size());
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, measurements, 
                    true);
            calibrator.setTime(timestamp);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessErrorWithExcessiveOverallNoise() throws WrongSizeException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var timestamp = new Date(createTimestamp(randomizer));
        final var nedPosition = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);

        final var nedC = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                ecefFrame, ecefFrame);

        reset();
        assertTrue(measurements.isEmpty());
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, mGeneratedMeasurement);
        assertEquals(0, reset);

        final var generator = new MagnetometerMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                timestamp, nedPosition, cnb, noiseRandomizer);

        assertEquals(1, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(1, error);

        final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);
        final var kb = new BodyKinematicsAndMagneticFluxDensity(trueKinematics, b);
        assertFalse(generator.process(kb));

        generator.reset();

        assertEquals(1, reset);

        assertTrue(generator.process(kb));
    }

    @Test
    void testProcessSkipStaticInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, 
            LockedException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var timestamp = new Date(createTimestamp(randomizer));
        var nedPosition = createPosition(randomizer);
        var cnb = generateBodyC(randomizer);

        var nedC = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(measurements.isEmpty());
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, mGeneratedMeasurement);
        assertEquals(0, reset);

        final var generator = new MagnetometerMeasurementsGenerator(this);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator, 
                timestamp, nedPosition, cnb, noiseRandomizer);

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);

        final var staticPeriodLength = generator.getMinStaticSamples() / 2;
        final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        nedPosition = nedFrame.getPosition();
        nedC = nedFrame.getCoordinateTransformation();
        cnb = nedC.inverseAndReturnNew();

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator, 
                timestamp, nedPosition, cnb, noiseRandomizer);

        assertEquals(1, staticIntervalDetected);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, true);

        assertEquals(1, dynamicIntervalDetected);
        assertEquals(1, staticIntervalSkipped);
    }

    @Test
    void testProcessSkipDynamicInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, 
            LockedException, IOException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final var timestamp = new Date(createTimestamp(randomizer));
        var nedPosition = createPosition(randomizer);
        var cnb = generateBodyC(randomizer);

        var nedC = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                ecefFrame, ecefFrame);

        reset();
        assertTrue(measurements.isEmpty());
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, mGeneratedMeasurement);
        assertEquals(0, reset);

        final var generator = new MagnetometerMeasurementsGenerator(this);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                timestamp, nedPosition, cnb, noiseRandomizer);

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);

        final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final var dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

        nedPosition = nedFrame.getPosition();
        nedC = nedFrame.getCoordinateTransformation();
        cnb = nedC.inverseAndReturnNew();

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                timestamp, nedPosition, cnb, noiseRandomizer);

        assertEquals(1, staticIntervalDetected);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, true);

        assertEquals(1, dynamicIntervalDetected);
        assertEquals(1, mDynamicIntervalSkipped);
    }

    @Override
    public void onInitializationStarted(final MagnetometerMeasurementsGenerator generator) {
        initializationStarted++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.INITIALIZING, generator.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final MagnetometerMeasurementsGenerator generator, final double baseNoiseLevel) {
        initializationCompleted++;
        checkLocked(generator);

        assertTrue(baseNoiseLevel > 0.0);
        assertEquals(baseNoiseLevel, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final var baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), baseNoiseLevel, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final var baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);

        assertTrue(generator.getThreshold() > 0.0);
        final var threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), generator.getThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Override
    public void onError(
            final MagnetometerMeasurementsGenerator generator, final TriadStaticIntervalDetector.ErrorReason reason) {
        error++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.FAILED, generator.getStatus());
    }

    @Override
    public void onStaticIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
        staticIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
        dynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onStaticIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
        staticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
        mDynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onGeneratedMeasurement(
            final MagnetometerMeasurementsGenerator generator,
            final StandardDeviationBodyMagneticFluxDensity measurement) {
        mGeneratedMeasurement++;
        measurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(final MagnetometerMeasurementsGenerator generator) {
        reset++;

        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
    }

    private void reset() {
        measurements.clear();

        initializationStarted = 0;
        initializationCompleted = 0;
        error = 0;
        staticIntervalDetected = 0;
        dynamicIntervalDetected = 0;
        staticIntervalSkipped = 0;
        mDynamicIntervalSkipped = 0;
        mGeneratedMeasurement = 0;
        reset = 0;
    }

    private void checkLocked(final MagnetometerMeasurementsGenerator generator) {
        assertTrue(generator.isRunning());
        assertThrows(LockedException.class, () -> generator.setMinStaticSamples(0));
        assertThrows(LockedException.class, () -> generator.setMaxDynamicSamples(0));
        assertThrows(LockedException.class, () -> generator.setListener(this));
        assertThrows(LockedException.class, () -> generator.setWindowSize(0));
        assertThrows(LockedException.class, () -> generator.setInitialStaticSamples(0));
        assertThrows(LockedException.class, () -> generator.setThresholdFactor(0.0));
        assertThrows(LockedException.class, () -> generator.setInstantaneousNoiseLevelFactor(0.0));
        assertThrows(LockedException.class, () -> generator.setBaseNoiseLevelAbsoluteThreshold(0.0));
        assertThrows(LockedException.class, () -> generator.setBaseNoiseLevelAbsoluteThreshold(null));
        assertThrows(LockedException.class, () -> generator.process(null));
        assertThrows(LockedException.class, generator::reset);
    }

    private static BodyMagneticFluxDensity generateB(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final Date timestamp, final NEDPosition position,
            final CoordinateTransformation cnb) {

        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic;
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

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final var latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
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

    private static void generateStaticSamples(
            final MagnetometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final IMUErrors errors, final Matrix hardIron, final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final Date timestamp,
            final NEDPosition nedPosition, final CoordinateTransformation cnb, final GaussianRandomizer noiseRandomizer)
            throws LockedException {
        final var random = new Random();
        final var measuredKinematics = new BodyKinematics();
        for (var i = 0; i < numSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final var kb = new BodyKinematicsAndMagneticFluxDensity(measuredKinematics, b);
            assertTrue(generator.process(kb));
        }
    }

    private static void generateDynamicSamples(
            final MagnetometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final Matrix hardIron, final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final Date timestamp,
            final NEDPosition nedPosition, final CoordinateTransformation cnb, final GaussianRandomizer noiseRandomizer,
            final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException, LockedException {

        final var deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var nedC = nedFrame.getCoordinateTransformation();

        final var roll = nedC.getRollEulerAngle();
        final var pitch = nedC.getPitchEulerAngle();
        final var yaw = nedC.getYawEulerAngle();

        var oldNedFrame = new NEDFrame(nedFrame);
        var newNedFrame = new NEDFrame();
        var oldEcefFrame = new ECEFFrame(ecefFrame);
        var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < numSamples; i++) {
            final var newRoll = oldRoll + deltaRoll;
            final var newPitch = oldPitch + deltaPitch;
            final var newYaw = oldYaw + deltaYaw;
            final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final var newEcefX = oldEcefX + deltaX;
            final var newEcefY = oldEcefY + deltaY;
            final var newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final var kb = new BodyKinematicsAndMagneticFluxDensity(measuredKinematics, b);
            assertTrue(generator.process(kb));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);
    }
}
