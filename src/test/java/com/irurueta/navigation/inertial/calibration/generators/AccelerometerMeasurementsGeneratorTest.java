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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccelerometerMeasurementsGeneratorTest implements AccelerometerMeasurementsGeneratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-8;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private int initializationStarted;
    private int initializationCompleted;
    private int error;
    private int staticIntervalDetected;
    private int dynamicIntervalDetected;
    private int staticIntervalSkipped;
    private int dynamicIntervalSkipped;
    private int generatedMeasurement;
    private int reset;

    private final List<StandardDeviationBodyKinematics> measurements = new ArrayList<>();

    @Test
    void testConstructor1() {
        final var generator = new AccelerometerMeasurementsGenerator();

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);
        final var timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
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
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
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
        final var generator = new AccelerometerMeasurementsGenerator(this);

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);
        final var timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
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
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, generator.getThreshold(), 0.0);
        final var threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(0.0, threshold1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Test
    void testGetSetTimeInterval1() throws LockedException {
        final var generator = new AccelerometerMeasurementsGenerator();

        // check default value
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);

        // set new value
        final var timeInterval = 2 * TIME_INTERVAL_SECONDS;
        generator.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, generator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeInterval2() throws LockedException {
        final var generator = new AccelerometerMeasurementsGenerator();

        // check default value
        final var timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final var timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        generator.setTimeInterval(timeInterval2);

        // check
        final var timeInterval3 = generator.getTimeIntervalAsTime();
        final var timeInterval4 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        final var timeInterval5 = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> generator.setTimeInterval(timeInterval5));
    }

    @Test
    void testGetSetMinStaticSamples() throws LockedException {
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(this, generator.getListener());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
        final var generator = new AccelerometerMeasurementsGenerator();

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
    void testProcessCalibrateAndResetWithNoise() throws WrongSizeException, 
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

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
            assertEquals(0, generatedMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors);

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, true);

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(measurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var initialBa = new Matrix(3, 1);
            final var initialMa = new Matrix(3, 3);
            final var calibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), measurements,
                    false, initialBa, initialMa);

            calibrator.calibrate();

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessErrorWithExcessiveOverallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

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
        assertEquals(0, generatedMeasurement);
        assertEquals(0, reset);

        final var generator = new AccelerometerMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

        assertEquals(1, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(1, error);

        assertFalse(generator.process(trueKinematics));

        generator.reset();

        assertEquals(1, reset);

        assertTrue(generator.process(trueKinematics));
    }

    @Test
    void testProcessSmallNoiseOnlyRotationAndCommonAxis() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, SMALL_ROOT_PSD, accelQuantLevel,
                gyroQuantLevel);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

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
            assertEquals(0, generatedMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors);

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, false);

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(measurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var initialBa = new Matrix(3, 1);
            final var initialMa = new Matrix(3, 3);
            final var calibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), measurements,
                    true, initialBa, initialMa);

            calibrator.calibrate();

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessSmallNoiseWithRotationAndPositionChange() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, CalibrationException, NotReadyException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, SMALL_ROOT_PSD, accelQuantLevel,
                gyroQuantLevel);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

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
            assertEquals(0, generatedMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (var i = 0; i < numMeasurements; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors);

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                        errors, true);

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(measurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var initialBa = new Matrix(3, 1);
            final var initialMa = new Matrix(3, 3);
            final var calibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), measurements,
                    true, initialBa, initialMa);

            calibrator.calibrate();

            final var estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessSkipStaticInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

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
        assertEquals(0, generatedMeasurement);
        assertEquals(0, reset);

        final var generator = new AccelerometerMeasurementsGenerator(this);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);

        final var staticPeriodLength = generator.getMinStaticSamples() / 2;
        final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors);

        assertEquals(1, staticIntervalDetected);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                true);

        assertEquals(1, dynamicIntervalDetected);
        assertEquals(1, staticIntervalSkipped);
    }

    @Test
    void testProcessSkipDynamicInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

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
        assertEquals(0, generatedMeasurement);
        assertEquals(0, reset);

        final var generator = new AccelerometerMeasurementsGenerator(this);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors);

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors);

        assertEquals(1, staticIntervalDetected);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                true);

        assertEquals(1, dynamicIntervalDetected);
        assertEquals(1, dynamicIntervalSkipped);
    }

    @Override
    public void onInitializationStarted(final AccelerometerMeasurementsGenerator generator) {
        initializationStarted++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.INITIALIZING, generator.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final AccelerometerMeasurementsGenerator generator, final double baseNoiseLevel) {
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
        assertEquals(baseNoiseLevel * Math.sqrt(generator.getTimeInterval()),
                generator.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getAccelerometerBaseNoiseLevelPsd(),
                Math.pow(generator.getAccelerometerBaseNoiseLevelRootPsd(), 2.0), SMALL_ABSOLUTE_ERROR);

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
            final AccelerometerMeasurementsGenerator generator, final TriadStaticIntervalDetector.ErrorReason reason) {
        error++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.FAILED, generator.getStatus());
    }

    @Override
    public void onStaticIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
        staticIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
        dynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onStaticIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
        staticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
        dynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onGeneratedMeasurement(
            final AccelerometerMeasurementsGenerator generator, final StandardDeviationBodyKinematics measurement) {
        generatedMeasurement++;
        measurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(final AccelerometerMeasurementsGenerator generator) {
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
        dynamicIntervalSkipped = 0;
        generatedMeasurement = 0;
        reset = 0;
    }

    private void checkLocked(final AccelerometerMeasurementsGenerator generator) {
        assertTrue(generator.isRunning());
        assertThrows(LockedException.class, () -> generator.setTimeInterval(0.0));
        final var timeInterval = new Time(1.0, TimeUnit.SECOND);
        assertThrows(LockedException.class, () -> generator.setTimeInterval(timeInterval));
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

    private static Matrix generateMaGeneral() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
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
            final AccelerometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final IMUErrors errors) throws LockedException {

        final var random = new Random();
        final var measuredKinematics = new BodyKinematics();
        for (var i = 0; i < numSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            assertTrue(generator.process(measuredKinematics));
        }
    }

    @SuppressWarnings("SameParameterValue")
    private static void generateDynamicSamples(
            final AccelerometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final boolean changePosition)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {

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

            assertTrue(generator.process(measuredKinematics));

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
