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
package com.irurueta.navigation.inertial.calibration.intervals;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccelerationTriadStaticIntervalDetectorTest implements AccelerationTriadStaticIntervalDetectorListener {

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

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-1;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-3;

    private static final double VERY_SMALL_ABSOLUTE_ERROR = 1e-8;

    private int initializationStarted;

    private int initializationCompleted;

    private int error;

    private int staticIntervalDetected;

    private int dynamicIntervalDetected;

    private int reset;

    private double errorAccumulatedNoiseLevel;

    private double errorInstantaneousNoiseLevel;

    private AccelerationTriadStaticIntervalDetector.ErrorReason errorReason;

    @Test
    void testConstructor1() {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, 
                detector.getInitialStaticSamples());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertNull(detector.getListener());
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);
        final var timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final var a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final var a3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final var a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final var a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final var a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final var a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final var a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final var a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final var a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final var a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final var a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final var a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, a13.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a13.getUnit());
        final var a14 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final var a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, a15.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a15.getUnit());
        final var a16 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final var triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad3.getUnit());
        final var triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final var a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, a17.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a17.getUnit());
        final var a18 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final var a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, a19.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a19.getUnit());
        final var a20 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final var a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, a21.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a21.getUnit());
        final var a22 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final var triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad5.getUnit());
        final var triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final var a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, a23.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a23.getUnit());
        final var a24 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final var a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, a25.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a25.getUnit());
        final var a26 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final var a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, a27.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a27.getUnit());
        final var a28 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final var triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad7.getUnit());
        final var triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    void testConstructor2() {
        final var detector = new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                detector.getInitialStaticSamples());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertSame(this, detector.getListener());
        final var timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final var a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final var a3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final var a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final var a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final var a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final var a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final var a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final var a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final var a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final var a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final var a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final var a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, a13.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a13.getUnit());
        final var a14 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final var a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, a15.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a15.getUnit());
        final var a16 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final var triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad3.getUnit());
        final var triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final var a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, a17.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a17.getUnit());
        final var a18 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final var a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, a19.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a19.getUnit());
        final var a20 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final var a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, a21.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a21.getUnit());
        final var a22 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final var triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad5.getUnit());
        final var triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final var a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, a23.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a23.getUnit());
        final var a24 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final var a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, a25.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a25.getUnit());
        final var a26 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final var a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, a27.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a27.getUnit());
        final var a28 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final var triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad7.getUnit());
        final var triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());

        // set new value
        detector.setWindowSize(3);

        // check
        assertEquals(3, detector.getWindowSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setWindowSize(1));
        assertThrows(IllegalArgumentException.class, () -> detector.setWindowSize(2));
    }

    @Test
    void testGetSetInitialStaticSamples() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, 
                detector.getInitialStaticSamples());

        // set new value
        detector.setInitialStaticSamples(2);

        // check
        assertEquals(2, detector.getInitialStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setInitialStaticSamples(1));
    }

    @Test
    void testGetSetThresholdFactor() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);

        // set new value
        detector.setThresholdFactor(1.0);

        // check
        assertEquals(1.0, detector.getThresholdFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setThresholdFactor(0.0));
    }

    @Test
    void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);

        // set new value
        detector.setInstantaneousNoiseLevelFactor(1.0);

        // check
        assertEquals(1.0, detector.getInstantaneousNoiseLevelFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setInstantaneousNoiseLevelFactor(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // set new value
        detector.setBaseNoiseLevelAbsoluteThreshold(1.0);

        // check
        assertEquals(1.0, detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setBaseNoiseLevelAbsoluteThreshold(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final var a1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());

        // set new value
        final var a2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final var a3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final var a4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(this, detector.getListener());
    }

    @Test
    void testGetSetTimeInterval1() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);

        // set new value
        final var timeInterval = 2 * TIME_INTERVAL_SECONDS;
        detector.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, detector.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeInterval2() throws LockedException {
        final var detector = new AccelerationTriadStaticIntervalDetector();

        final var timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        final var timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        detector.setTimeInterval(timeInterval2);

        final var timeInterval3 = detector.getTimeIntervalAsTime();
        final var timeInterval4 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1() throws WrongSizeException,
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

        final var accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final var lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, reset);

        final var detector = new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AccelerationTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        assertEquals(detector.getAccumulatedStdX(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdY(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdZ(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(accelNoiseStd, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        final var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final var deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        for (var i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, lastStaticTriad.getUnit());
        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2() throws WrongSizeException,
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

        final var accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final var lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, reset);

        final var detector = new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AccelerationTriad();
        final var aX = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var aY = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var aZ = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var random = new Random();

        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        assertEquals(accelNoiseStd, detector.getAccumulatedStdX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(accelNoiseStd, detector.getAccumulatedStdY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(accelNoiseStd, detector.getAccumulatedStdZ(), SMALL_ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(accelNoiseStd, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        final var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final var deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        for (var i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, lastStaticTriad.getUnit());
        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3() throws WrongSizeException,
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

        final var accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final var lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, reset);

        final var detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AccelerationTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(), Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        assertEquals(accelNoiseStd, detector.getAccumulatedStdX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(accelNoiseStd, detector.getAccumulatedStdY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(accelNoiseStd, detector.getAccumulatedStdZ(), SMALL_ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(stdZ1.getValue().doubleValue(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        final var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final var deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        for (var i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, lastStaticTriad.getUnit());
        assertEquals(lastStaticTriad.getValueX(), detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(), detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(), detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(detector.getAccumulatedAvgXAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(detector.getAccumulatedAvgYAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(detector.getAccumulatedAvgZAsMeasurement(),
                ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(), ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    void testProcessWithExcessiveOverallNoiseDuringInitialization() throws WrongSizeException,
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
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, reset);

        final var detector = new AccelerationTriadStaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AccelerationTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));

            if (error != 0) {
                break;
            }
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, error);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(errorAccumulatedNoiseLevel > 0.0);
        assertTrue(errorInstantaneousNoiseLevel > 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED,
                errorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Test
    void testProcessWithSuddenMotionDuringInitialization() throws WrongSizeException,
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
        assertEquals(0, initializationStarted);
        assertEquals(0, initializationCompleted);
        assertEquals(0, error);
        assertEquals(0, staticIntervalDetected);
        assertEquals(0, dynamicIntervalDetected);
        assertEquals(0, reset);

        final var detector = new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final var initialStaticSamples = detector.getInitialStaticSamples();
        var periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        var halfInitialStaticSamples = initialStaticSamples / 2;

        // add some samples while keeping accelerometer body static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AccelerationTriad();
        final var random = new Random();
        for (var i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, initializationStarted);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());

        // then add samples with motion
        final var deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        for (var i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();

            if (error != 0) {
                break;
            }
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, error);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(errorAccumulatedNoiseLevel > 0.0);
        assertTrue(errorInstantaneousNoiseLevel > 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,
                errorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final AccelerationTriadStaticIntervalDetector detector) {
        initializationStarted++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final AccelerationTriadStaticIntervalDetector detector, final double baseNoiseLevel) {
        initializationCompleted++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
    }

    @Override
    public void onError(
            final AccelerationTriadStaticIntervalDetector detector, final double accumulatedNoiseLevel,
            final double instantaneousNoiseLevel, final TriadStaticIntervalDetector.ErrorReason reason) {
        error++;
        errorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        errorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        errorReason = reason;
        checkLocked(detector);
    }

    @Override
    public void onStaticIntervalDetected(final AccelerationTriadStaticIntervalDetector detector,
                                         final double instantaneousAvgX,
                                         final double instantaneousAvgY,
                                         final double instantaneousAvgZ,
                                         final double instantaneousStdX,
                                         final double instantaneousStdY,
                                         final double instantaneousStdZ) {
        staticIntervalDetected++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final var a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final var a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final var a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final var a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final var a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final var avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final var a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final var a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final var a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final var a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final var a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final var a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final var stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onDynamicIntervalDetected(final AccelerationTriadStaticIntervalDetector detector,
                                          final double instantaneousAvgX,
                                          final double instantaneousAvgY,
                                          final double instantaneousAvgZ,
                                          final double instantaneousStdX,
                                          final double instantaneousStdY,
                                          final double instantaneousStdZ,
                                          final double accumulatedAvgX,
                                          final double accumulatedAvgY,
                                          final double accumulatedAvgZ,
                                          final double accumulatedStdX,
                                          final double accumulatedStdY,
                                          final double accumulatedStdZ) {
        dynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final var ax1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(accumulatedAvgX, ax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, ax1.getUnit());
        final var ax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(ax2);
        assertEquals(ax1, ax2);

        final var ay1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(accumulatedAvgY, ay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, ay1.getUnit());
        final var ay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(ay2);
        assertEquals(ay1, ay2);

        final var az1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(accumulatedAvgZ, az1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, az1.getUnit());
        final var az2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(az2);
        assertEquals(az1, az2);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(accumulatedAvgX, triad1.getValueX(), 0.0);
        assertEquals(accumulatedAvgY, triad1.getValueY(), 0.0);
        assertEquals(accumulatedAvgZ, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        final var triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final var a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final var a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final var a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final var a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final var a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final var avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final var a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final var a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final var a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final var a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final var a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final var a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final var stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final AccelerationTriadStaticIntervalDetector detector) {
        reset++;
        checkLocked(detector);
    }

    private void reset() {
        initializationStarted = 0;
        initializationCompleted = 0;
        error = 0;
        staticIntervalDetected = 0;
        dynamicIntervalDetected = 0;
        reset = 0;
    }

    private void checkLocked(final AccelerationTriadStaticIntervalDetector detector) {
        assertTrue(detector.isRunning());
        assertThrows(LockedException.class, () -> detector.setWindowSize(0));
        assertThrows(LockedException.class, () -> detector.setInitialStaticSamples(0));
        assertThrows(LockedException.class, () -> detector.setThresholdFactor(0.0));
        assertThrows(LockedException.class, () -> detector.setInstantaneousNoiseLevelFactor(0.0));
        assertThrows(LockedException.class, () -> detector.setBaseNoiseLevelAbsoluteThreshold(0.0));
        assertThrows(LockedException.class, () -> detector.setListener(this));
        assertThrows(LockedException.class, () -> detector.setTimeInterval(0.0));
        final var timeInterval = new Time(1.0, TimeUnit.DAY);
        assertThrows(LockedException.class, () -> detector.setTimeInterval(timeInterval));
        final var triad = new AccelerationTriad();
        assertThrows(LockedException.class, () -> detector.process(triad));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(LockedException.class, () -> detector.process(a, a, a));
        assertThrows(LockedException.class, () -> detector.process(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, detector::reset);
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
}
