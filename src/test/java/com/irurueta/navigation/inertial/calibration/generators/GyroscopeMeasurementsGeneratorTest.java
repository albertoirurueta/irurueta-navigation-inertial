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
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class GyroscopeMeasurementsGeneratorTest implements GyroscopeMeasurementsGeneratorListener {

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

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private int mInitializationStarted;

    private int mInitializationCompleted;

    private int mError;

    private int mStaticIntervalDetected;

    private int mDynamicIntervalDetected;

    private int mStaticIntervalSkipped;

    private int mDynamicIntervalSkipped;

    private int mGeneratedMeasurement;

    private int mReset;

    private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mSequences = new ArrayList<>();

    private final List<StandardDeviationBodyKinematics> mMeasurements = new ArrayList<>();

    private final AccelerometerMeasurementsGeneratorListener mAccelerometerGeneratorListener =
            new AccelerometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final AccelerometerMeasurementsGenerator generator) {
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerMeasurementsGenerator generator, final double baseNoiseLevel) {
                }

                @Override
                public void onError(
                        final AccelerometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                }

                @Override
                public void onStaticIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
                }

                @Override
                public void onDynamicIntervalDetected(final AccelerometerMeasurementsGenerator generator) {
                }

                @Override
                public void onStaticIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
                }

                @Override
                public void onDynamicIntervalSkipped(final AccelerometerMeasurementsGenerator generator) {
                }

                @Override
                public void onGeneratedMeasurement(
                        final AccelerometerMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    mMeasurements.add(measurement);
                }

                @Override
                public void onReset(final AccelerometerMeasurementsGenerator generator) {
                }
            };

    @Test
    public void testConstructor1() {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(),
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, errorThreshold1.getUnit());
        final Acceleration errorThreshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, baseNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final Acceleration baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, generator.getThreshold(), 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(0.0, threshold1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final AngularSpeedTriad stdAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, gyroNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
    }

    @Test
    public void testConstructor2() {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(), generator.getBaseNoiseLevelAbsoluteThreshold(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, errorThreshold1.getUnit());
        final Acceleration errorThreshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, baseNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final Acceleration baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, generator.getThreshold(), 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(0.0, threshold1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final AngularSpeedTriad stdAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, gyroNoiseLevel1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

        // check default value
        assertEquals(TIME_INTERVAL_SECONDS, generator.getTimeInterval(), 0.0);

        // set new value
        final double timeInterval = 2 * TIME_INTERVAL_SECONDS;
        generator.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, generator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> generator.setTimeInterval(-1.0));
    }

    @Test
    public void testGetSetTimeInterval2() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

        // check default value
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final Time timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        generator.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = generator.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        final Time timeInterval5 = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> generator.setTimeInterval(timeInterval5));
    }

    @Test
    public void testGetSetMinStaticSamples() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetMaxDynamicSamples() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetListener() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(this, generator.getListener());
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetInitialStaticSamples() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetThresholdFactor() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

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
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {
        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final Acceleration a1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());

        // set new value
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        generator.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final Acceleration a3 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration a4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    public void testProcessCalibrateAndResetWithNoiseMaCommonAxisAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            boolean failed = false;
            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                if (mDynamicIntervalDetected != i + 1) {
                    failed = true;
                    break;
                }
                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mSequences.size(), i);
            }

            if (failed) {
                continue;
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseMaGeneralAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mSequences.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements, false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseMaCommonAxisAndNoGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mSequences.size(), i);
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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
    public void testProcessCalibrateAndResetSmallNoiseMaGeneralAndWithGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(i + 1, mDynamicIntervalDetected);
                assertEquals(i, mSequences.size());
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences,
                    true, true, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences,
                    true, true, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements,
                            false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseMaCommonAxisAndWithGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(i + 1, mDynamicIntervalDetected);
                assertEquals(i, mSequences.size());
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences, true,
                    true, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences, true,
                    true, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessSkipStaticInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InvalidRotationMatrixException,
            RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(mSequences.isEmpty());
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mGeneratedMeasurement);
        assertEquals(0, mReset);

        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

        final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                mAccelerometerGeneratorListener);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors, random,
                0);

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);

        final int staticPeriodLength = generator.getMinStaticSamples() / 2;
        final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        int start = initialStaticSamples;
        // generate static samples
        generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors, random,
                start);
        start += staticPeriodLength;

        assertEquals(1, mStaticIntervalDetected);

        // generate dynamic samples
        assertNotNull(generateDynamicSamples(generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics,
                randomizer, ecefFrame, nedFrame, errors, random, start, false));

        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(1, mStaticIntervalSkipped);
    }

    @Test
    public void testProcessSkipDynamicInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InvalidRotationMatrixException,
            RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = new Matrix(3, 3);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0);

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

            int start = initialStaticSamples;
            // generate static samples
            generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                    random, start);
            start += staticPeriodLength;

            assertEquals(1, mStaticIntervalDetected);

            // generate dynamic samples
            assertNotNull(generateDynamicSamples(generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics,
                    randomizer, ecefFrame, nedFrame, errors, random, start, false));

            if (mDynamicIntervalDetected != 1) {
                continue;
            }
            assertEquals(1, mDynamicIntervalDetected);
            assertEquals(1, mDynamicIntervalSkipped);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessErrorWithExcessiveOverallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(mSequences.isEmpty());
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mGeneratedMeasurement);
        assertEquals(0, mReset);

        final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        final AccelerometerMeasurementsGenerator accelerometerGenerator =
                new AccelerometerMeasurementsGenerator(mAccelerometerGeneratorListener);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors, random,
                0);

        assertEquals(1, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(1, mError);

        final TimedBodyKinematics timeKinematics = new TimedBodyKinematics();
        timeKinematics.setKinematics(trueKinematics);
        assertFalse(generator.process(timeKinematics));

        generator.reset();

        assertEquals(1, mReset);

        assertTrue(generator.process(timeKinematics));
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseWithRotationAndPositionChangeMaCommonAxisAndNoGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mSequences.isEmpty());
            assertEquals(0, mInitializationStarted);
            assertEquals(0, mInitializationCompleted);
            assertEquals(0, mError);
            assertEquals(0, mStaticIntervalDetected);
            assertEquals(0, mDynamicIntervalDetected);
            assertEquals(0, mGeneratedMeasurement);
            assertEquals(0, mReset);

            final GyroscopeMeasurementsGenerator generator = new GyroscopeMeasurementsGenerator(this);

            final AccelerometerMeasurementsGenerator accelerometerGenerator = new AccelerometerMeasurementsGenerator(
                    mAccelerometerGeneratorListener);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
            if (!generateStaticSamples(generator, accelerometerGenerator, initialStaticSamples, trueKinematics, errors,
                    random, 0)) {
                continue;
            }

            assertEquals(1, mInitializationStarted);
            assertEquals(1, mInitializationCompleted);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, accelerometerGenerator, staticPeriodLength, trueKinematics, errors,
                        random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = generateDynamicSamples(
                        generator, accelerometerGenerator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame,
                        nedFrame, errors, random, start, true);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(i + 1, mDynamicIntervalDetected);
                assertEquals(i, mSequences.size());
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(1, mReset);
            assertEquals(0, mError);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator = new EasyGyroscopeCalibrator(sequences,
                    true, false, initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator(mSequences, true,
                    false, initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), mMeasurements, true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onInitializationStarted(final GyroscopeMeasurementsGenerator generator) {
        mInitializationStarted++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.INITIALIZING, generator.getStatus());
    }

    @Override
    public void onInitializationCompleted(final GyroscopeMeasurementsGenerator generator, final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(generator);

        assertTrue(baseNoiseLevel > 0.0);
        assertEquals(baseNoiseLevel, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), baseNoiseLevel, 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baseNoiseLevel1.getUnit());
        final Acceleration baseNoiseLevel2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        final double sqrtTimeInterval = Math.sqrt(generator.getTimeInterval());
        assertEquals(baseNoiseLevel * sqrtTimeInterval, generator.getAccelerometerBaseNoiseLevelRootPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getAccelerometerBaseNoiseLevelPsd(),
                Math.pow(generator.getAccelerometerBaseNoiseLevelRootPsd(), 2.0), SMALL_ABSOLUTE_ERROR);

        assertTrue(generator.getThreshold() > 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), generator.getThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
        final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);

        final AngularSpeedTriad stdAngularSpeed1 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);

        final double gyroNoiseLevel = generator.getGyroscopeBaseNoiseLevel();
        assertTrue(gyroNoiseLevel > 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(gyroNoiseLevel1.getValue().doubleValue(), gyroNoiseLevel, 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, gyroNoiseLevel1.getUnit());
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);

        assertEquals(stdAngularSpeed1.getNorm(), gyroNoiseLevel, 0.0);

        assertEquals(gyroNoiseLevel * sqrtTimeInterval, generator.getGyroscopeBaseNoiseLevelRootPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getGyroscopeBaseNoiseLevelPsd(),
                Math.pow(generator.getGyroscopeBaseNoiseLevelRootPsd(), 2.0), SMALL_ABSOLUTE_ERROR);
    }

    @Override
    public void onError(
            final GyroscopeMeasurementsGenerator generator, final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.FAILED, generator.getStatus());
    }

    @Override
    public void onStaticIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
        mStaticIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.STATIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalDetected(final GyroscopeMeasurementsGenerator generator) {
        mDynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onStaticIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
        mStaticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onDynamicIntervalSkipped(final GyroscopeMeasurementsGenerator generator) {
        mDynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, generator.getStatus());
    }

    @Override
    public void onGeneratedMeasurement(
            final GyroscopeMeasurementsGenerator generator,
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
        mGeneratedMeasurement++;
        mSequences.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(final GyroscopeMeasurementsGenerator generator) {
        mReset++;

        assertEquals(TriadStaticIntervalDetector.Status.IDLE, generator.getStatus());
    }

    private void reset() {
        mSequences.clear();
        mMeasurements.clear();

        mInitializationStarted = 0;
        mInitializationCompleted = 0;
        mError = 0;
        mStaticIntervalDetected = 0;
        mDynamicIntervalDetected = 0;
        mStaticIntervalSkipped = 0;
        mDynamicIntervalSkipped = 0;
        mGeneratedMeasurement = 0;
        mReset = 0;
    }

    private void checkLocked(final GyroscopeMeasurementsGenerator generator) {
        assertTrue(generator.isRunning());
        assertThrows(LockedException.class, () -> generator.setTimeInterval(0.0));
        final Time timeInterval = new Time(1.0, TimeUnit.SECOND);
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
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMaGeneral() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        final double tmp = DEG_TO_RAD / (3600 * 9.80665);
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

    private static boolean generateStaticSamples(
            final GyroscopeMeasurementsGenerator generator,
            final AccelerometerMeasurementsGenerator accelerometerGenerator, final int numSamples,
            final BodyKinematics trueKinematics, final IMUErrors errors, final Random random,
            final int startSample) throws LockedException {

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            final boolean result1 = generator.process(timedMeasuredKinematics);
            final boolean result2 = accelerometerGenerator.process(measuredKinematics);

            if (!result1 || !result2) {
                return false;
            }
        }

        return true;
    }

    @SuppressWarnings("SameParameterValue")
    private static BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> generateDynamicSamples(
            final GyroscopeMeasurementsGenerator generator,
            final AccelerometerMeasurementsGenerator accelerometerGenerator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final Random random, final int startSample,
            final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException, RotationException {

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final double deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final Quaternion beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        final NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        final NEDFrame newNedFrame = new NEDFrame();
        final ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        final ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = new BodyKinematicsSequence<>();
        sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                new BodyKinematicsSequence<>();
        final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();

        final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList = new ArrayList<>();

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final double progress = (double) i / (double) numSamples;

            final double newRoll = oldRoll + interpolate(deltaRoll, progress);
            final double newPitch = oldPitch + interpolate(deltaPitch, progress);
            final double newYaw = oldYaw + interpolate(deltaYaw, progress);
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + interpolate(deltaX, progress);
            final double newEcefY = oldEcefY + interpolate(deltaY, progress);
            final double newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            assertTrue(generator.process(timedMeasuredKinematics));
            assertTrue(accelerometerGenerator.process(measuredKinematics));

            final StandardDeviationTimedBodyKinematics trueTimedKinematics = new StandardDeviationTimedBodyKinematics(
                    new BodyKinematics(trueKinematics), timestampSeconds, specificForceStandardDeviation,
                    angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(new BodyKinematics(measuredKinematics), timestampSeconds,
                            specificForceStandardDeviation, angularRateStandardDeviation);
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

        final Quaternion afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ);

        final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);

        final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final double afterMeanFx = measuredAfterGravityKinematics.getFx();
        final double afterMeanFy = measuredAfterGravityKinematics.getFy();
        final double afterMeanFz = measuredAfterGravityKinematics.getFz();

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
