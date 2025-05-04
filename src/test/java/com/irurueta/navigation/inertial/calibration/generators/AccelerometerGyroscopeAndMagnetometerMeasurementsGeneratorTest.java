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
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.*;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorTest implements
        AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;
    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double SMALL_MAGNETOMETER_NOISE_STD = 1e-12;
    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;

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

    private int initializationStarted;
    private int initializationCompleted;
    private int error;
    private int staticIntervalDetected;
    private int dynamicIntervalDetected;
    private int staticIntervalSkipped;
    private int dynamicIntervalSkipped;
    private int generatedAccelerometerMeasurement;
    private int generatedGyroscopeMeasurement;
    private int generatedMagnetometerMeasurement;
    private int reset;

    private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> gyroscopeMeasurements =
            new ArrayList<>();

    private final List<StandardDeviationBodyKinematics> accelerometerMeasurements = new ArrayList<>();

    private final List<StandardDeviationBodyMagneticFluxDensity> magnetometerMeasurements = new ArrayList<>();

    @Test
    void testConstructor1() {
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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

        final var avgAngularSpeed1 = new AngularSpeedTriad();
        final var avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final var avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final var stdAngularSpeed1 = new AngularSpeedTriad();
        final var stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final var stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final var gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, gyroNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final var gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
    }

    @Test
    void testConstructor2() {
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

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

        final var avgAngularSpeed1 = new AngularSpeedTriad();
        final var avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final var avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final var stdAngularSpeed1 = new AngularSpeedTriad();
        final var stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final var stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final var gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, gyroNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final var gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
    }

    @Test
    void testGetSetTimeInterval1() throws LockedException {
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(this, generator.getListener());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator();

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
    void testProcessCalibrateAndResetWithNoiseMaCommonAxisAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final var timestamp = new Date(createTimestamp(randomizer));

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

            var failed = false;
            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer, start);
                start += staticPeriodLength;

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                final var sequence = generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer,
                        ecefFrame, nedFrame, errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb,
                        noiseRandomizer, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                if (dynamicIntervalDetected != i + 1 || accelerometerMeasurements.size() != i + 1
                        || gyroscopeMeasurements.size() != i) {
                    failed = true;
                    break;
                }

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(accelerometerMeasurements.size(), i + 1);
                assertEquals(gyroscopeMeasurements.size(), i);
            }

            if (failed) {
                continue;
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var trueCalibrator = new EasyGyroscopeCalibrator(sequences, true, 
                    false, initialBg, initialMg, initialGg, ba, ma);
            final var calibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final var estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), 
                    accelerometerMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var magnetometerCalibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, 
                    magnetometerMeasurements, true);
            magnetometerCalibrator.setTime(timestamp);

            try {
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!bg.equals(estimatedBgTrue, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

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
    void testProcessCalibrateAndResetSmallNoiseMaGeneralAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
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

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer, start);
                start += staticPeriodLength;

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                final var sequence = generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer,
                        ecefFrame, nedFrame, errors, hardIron, mm, wmmEstimator,timestamp, nedPosition, cnb, 
                        noiseRandomizer, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(accelerometerMeasurements.size(), i + 1);
                assertEquals(gyroscopeMeasurements.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);
            final var calibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final var estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), 
                    accelerometerMeasurements, false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var magnetometerCalibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, 
                    magnetometerMeasurements, true);
            magnetometerCalibrator.setTime(timestamp);

            try {
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!bg.equals(estimatedBgTrue, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessCalibrateAndResetSmallNoiseMaCommonAxisAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, 
            InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
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

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer, start);
                start += staticPeriodLength;

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                final var sequence = generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer,
                        ecefFrame, nedFrame, errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, 
                        noiseRandomizer, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(accelerometerMeasurements.size(), i + 1);
                assertEquals(gyroscopeMeasurements.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);
            final var calibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final var estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), 
                    accelerometerMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var magnetometerCalibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, 
                    magnetometerMeasurements, true);
            magnetometerCalibrator.setTime(timestamp);

            try {
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = magnetometerCalibrator.getEstimatedMm();


            if (!bg.equals(estimatedBgTrue, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessCalibrateAndResetSmallNoiseMaGeneralAndWithGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, 
            InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaGeneral();
        final var mg = generateMg();
        final var gg = generateGg();

        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
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

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer, start);
                start += staticPeriodLength;

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                final var sequence = generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer,
                        ecefFrame, nedFrame, errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, 
                        noiseRandomizer, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(accelerometerMeasurements.size(), i + 1);
                assertEquals(gyroscopeMeasurements.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    true, initialBg, initialMg, initialGg, ba, ma);
            final var calibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    true, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final var estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(),
                    accelerometerMeasurements, false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var magnetometerCalibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, 
                    magnetometerMeasurements, true);
            magnetometerCalibrator.setTime(timestamp);

            try {
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, SMALL_ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR));

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testProcessSkipStaticInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

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

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var cnb = nedC.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                ecefFrame, ecefFrame);

        reset();
        assertTrue(accelerometerMeasurements.isEmpty());
        assertTrue(gyroscopeMeasurements.isEmpty());
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, generatedAccelerometerMeasurement);
        assertEquals(0, generatedGyroscopeMeasurement);
        assertEquals(0, generatedMagnetometerMeasurement);
        assertEquals(0, reset);

        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                timestamp, nedPosition, cnb, noiseRandomizer, 0);

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);

        final var staticPeriodLength = generator.getMinStaticSamples() / 2;
        final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        var start = initialStaticSamples;
        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator, 
                timestamp, nedPosition, cnb, noiseRandomizer, start);
        start += staticPeriodLength;

        assertEquals(1, staticIntervalDetected);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, start, false);

        assertEquals(1, dynamicIntervalDetected);
        assertEquals(1, staticIntervalSkipped);
    }

    @Test
    void testProcessSkipDynamicInterval() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, InvalidRotationMatrixException, IOException, RotationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var bg = generateBg();
            final var ma = generateMaCommonAxis();
            final var mg = generateMg();
            final var gg = new Matrix(3, 3);

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

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

            var start = initialStaticSamples;
            // generate static samples
            generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, start);
            start += staticPeriodLength;

            assertEquals(1, staticIntervalDetected);

            // generate dynamic samples
            generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                    errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, noiseRandomizer, start, 
                    false);

            if (dynamicIntervalDetected != 1) {
                continue;
            }
            assertEquals(1, dynamicIntervalDetected);
            assertEquals(1, dynamicIntervalSkipped);

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
        final var gg = new Matrix(3, 3);

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

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var cnb = nedC.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                ecefFrame, ecefFrame);

        reset();
        assertTrue(accelerometerMeasurements.isEmpty());
        assertTrue(gyroscopeMeasurements.isEmpty());
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, generatedAccelerometerMeasurement);
        assertEquals(0, generatedGyroscopeMeasurement);
        assertEquals(0, generatedMagnetometerMeasurement);
        assertEquals(0, reset);

        final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                timestamp, nedPosition, cnb, noiseRandomizer, 0);

        assertEquals(1, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(1, error);

        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();
        final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

        tkb.setKinematics(trueKinematics);
        tkb.setMagneticFluxDensity(b);
        tkb.setTimestampSeconds(TIME_INTERVAL_SECONDS);

        assertFalse(generator.process(tkb));

        generator.reset();

        assertEquals(1, reset);

        assertTrue(generator.process(tkb));
    }

    @Test
    void testProcessCalibrateAndResetSmallNoiseWithRotationAndPositionChangeMaCommonAxisAndNoGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, IOException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = new Matrix(3, 3);

        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
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

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final var nedPosition = new NEDPosition(latitude, longitude, height);

            final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, 
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final var cnb = nedC.inverseAndReturnNew();

            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                    ecefFrame, ecefFrame);

            reset();
            assertTrue(accelerometerMeasurements.isEmpty());
            assertTrue(gyroscopeMeasurements.isEmpty());
            assertEquals(0, initializationStarted);
            assertEquals(0, initializationCompleted);
            assertEquals(0, error);
            assertEquals(0, staticIntervalDetected);
            assertEquals(0, dynamicIntervalDetected);
            assertEquals(0, generatedAccelerometerMeasurement);
            assertEquals(0, generatedGyroscopeMeasurement);
            assertEquals(0, generatedMagnetometerMeasurement);
            assertEquals(0, reset);

            final var generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    timestamp, nedPosition, cnb, noiseRandomizer, 0);

            assertEquals(1, initializationStarted);
            assertEquals(1, initializationCompleted);

            final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var n = Math.max(numSequences + 1, numMeasurements);

            final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final var sequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

            var start = initialStaticSamples;
            for (var i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                        timestamp, nedPosition, cnb, noiseRandomizer, start);
                start += staticPeriodLength;

                assertEquals(staticIntervalDetected, i + 1);

                // generate dynamic samples
                final var sequence = generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer,
                        ecefFrame, nedFrame, errors, hardIron, mm, wmmEstimator, timestamp, nedPosition, cnb, 
                        noiseRandomizer, start, true);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(dynamicIntervalDetected, i + 1);
                assertEquals(accelerometerMeasurements.size(), i + 1);
                assertEquals(gyroscopeMeasurements.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, reset);
            assertEquals(0, error);

            final var initialMg = new Matrix(3, 3);
            final var initialGg = new Matrix(3, 3);
            final var trueCalibrator = new EasyGyroscopeCalibrator(sequences, true, 
                    false, initialBg, initialMg, initialGg, ba, ma);
            final var calibrator = new EasyGyroscopeCalibrator(gyroscopeMeasurements, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final var estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final var estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMg = calibrator.getEstimatedMg();
            final var estimatedGg = calibrator.getEstimatedGg();

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), 
                    accelerometerMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final var estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final var magnetometerCalibrator = new KnownPositionAndInstantMagnetometerCalibrator(nedPosition, 
                    magnetometerMeasurements, true);
            magnetometerCalibrator.setTime(timestamp);

            try {
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final var estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, SMALL_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onInitializationStarted(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        initializationStarted++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.INITIALIZING, generator.getStatus());
    }

    @Override
    public void onInitializationCompleted(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final double accelerometerBaseNoiseLevel) {
        initializationCompleted++;
        checkLocked(generator);

        assertTrue(accelerometerBaseNoiseLevel > 0.0);
        assertEquals(accelerometerBaseNoiseLevel, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final var baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), accelerometerBaseNoiseLevel, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final var baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        final var sqrtTimeInterval = Math.sqrt(generator.getTimeInterval());
        assertEquals(accelerometerBaseNoiseLevel * sqrtTimeInterval,
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

        final var avgAngularSpeed1 = generator.getInitialAvgAngularSpeedTriad();
        final var avgAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);

        final var stdAngularSpeed1 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final var stdAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);

        final var gyroNoiseLevel = generator.getGyroscopeBaseNoiseLevel();
        assertTrue(gyroNoiseLevel > 0.0);
        final var gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(gyroNoiseLevel1.getValue().doubleValue(), gyroNoiseLevel, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final var gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);

        assertEquals(stdAngularSpeed1.getNorm(), gyroNoiseLevel, 0.0);

        assertEquals(gyroNoiseLevel * sqrtTimeInterval, generator.getGyroscopeBaseNoiseLevelRootPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getGyroscopeBaseNoiseLevelPsd(), Math.pow(generator.getGyroscopeBaseNoiseLevelRootPsd(),
                        2.0), SMALL_ABSOLUTE_ERROR);
    }

    @Override
    public void onError(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final TriadStaticIntervalDetector.ErrorReason reason) {
        error++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.FAILED, generator.getStatus());
    }

    @Override
    public void onStaticIntervalDetected(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        staticIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalDetected(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        dynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onStaticIntervalSkipped(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        staticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalSkipped(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        dynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onGeneratedAccelerometerMeasurement(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final StandardDeviationBodyKinematics measurement) {
        generatedAccelerometerMeasurement++;
        accelerometerMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onGeneratedGyroscopeMeasurement(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
        generatedGyroscopeMeasurement++;
        gyroscopeMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onGeneratedMagnetometerMeasurement(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final StandardDeviationBodyMagneticFluxDensity measurement) {
        generatedMagnetometerMeasurement++;
        magnetometerMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
        reset++;

        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
    }

    private void reset() {
        gyroscopeMeasurements.clear();
        accelerometerMeasurements.clear();
        magnetometerMeasurements.clear();

        initializationStarted = 0;
        initializationCompleted = 0;
        error = 0;
        staticIntervalDetected = 0;
        dynamicIntervalDetected = 0;
        staticIntervalSkipped = 0;
        dynamicIntervalSkipped = 0;
        generatedAccelerometerMeasurement = 0;
        generatedGyroscopeMeasurement = 0;
        generatedMagnetometerMeasurement = 0;
        reset = 0;
    }

    private void checkLocked(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
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

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
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
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
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
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Matrix hardIron,
            final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final Date timestamp,
            final NEDPosition nedPosition,
            final CoordinateTransformation cnb,
            final GaussianRandomizer noiseRandomizer,
            final int startSample) throws LockedException {

        final var random = new Random();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();
        final var measuredKinematics = new BodyKinematics();
        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);

            tkb.setKinematics(measuredKinematics);
            tkb.setMagneticFluxDensity(b);
            tkb.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            assertTrue(generator.process(tkb));
        }
    }

    @SuppressWarnings("SameParameterValue")
    private static BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> generateDynamicSamples(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final Matrix hardIron, final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final Date timestamp,
            final NEDPosition nedPosition, final CoordinateTransformation cnb, final GaussianRandomizer noiseRandomizer,
            final int startSample, final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException,
            LockedException, InvalidRotationMatrixException, RotationException {

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

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

        final var beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

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

        final var random = new Random();
        final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();

        final var measuredTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();

        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();
        final var measuredKinematics = new BodyKinematics();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final var progress = (double) i / (double) numSamples;

            final var newRoll = oldRoll + interpolate(deltaRoll, progress);
            final var newPitch = oldPitch + interpolate(deltaPitch, progress);
            final var newYaw = oldYaw + interpolate(deltaYaw, progress);
            final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final var newEcefX = oldEcefX + interpolate(deltaX, progress);
            final var newEcefY = oldEcefY + interpolate(deltaY, progress);
            final var newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);

            tkb.setKinematics(measuredKinematics);
            tkb.setMagneticFluxDensity(b);
            tkb.setTimestampSeconds(timestampSeconds);

            assertTrue(generator.process(tkb));

            final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(new BodyKinematics(trueKinematics),
                    timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            final var measuredTimedKinematics = new StandardDeviationTimedBodyKinematics(
                    new BodyKinematics(measuredKinematics), timestampSeconds, specificForceStandardDeviation,
                    angularRateStandardDeviation);
            measuredTimedKinematicsList.add(measuredTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final var afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ);

        final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        final var newCnb = newNedC.inverseAndReturnNew();

        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);
        cnb.copyFrom(newCnb);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);

        final var measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final var afterMeanFx = measuredAfterGravityKinematics.getFx();
        final var afterMeanFy = measuredAfterGravityKinematics.getFy();
        final var afterMeanFz = measuredAfterGravityKinematics.getFz();

        sequence.setItems(measuredTimedKinematicsList);
        sequence.setAfterMeanSpecificForceCoordinates(afterMeanFx, afterMeanFy, afterMeanFz);

        return sequence;
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private static double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }
}
