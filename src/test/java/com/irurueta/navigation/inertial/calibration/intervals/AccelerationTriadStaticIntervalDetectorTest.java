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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AccelerationTriadStaticIntervalDetectorTest implements AccelerationTriadStaticIntervalDetectorListener {

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

    private int mInitializationStarted;

    private int mInitializationCompleted;

    private int mError;

    private int mStaticIntervalDetected;

    private int mDynamicIntervalDetected;

    private int mReset;

    private double mErrorAccumulatedNoiseLevel;

    private double mErrorInstantaneousNoiseLevel;

    private AccelerationTriadStaticIntervalDetector.ErrorReason mErrorReason;

    @Test
    public void testConstructor1() {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final Acceleration a3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final Acceleration a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final Acceleration a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final Acceleration a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final Acceleration a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final Acceleration a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final Acceleration a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final Acceleration a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final Acceleration a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final Acceleration a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final Acceleration a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, a13.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a13.getUnit());
        final Acceleration a14 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final Acceleration a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, a15.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a15.getUnit());
        final Acceleration a16 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final AccelerationTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad3.getUnit());
        final AccelerationTriad triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final Acceleration a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, a17.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a17.getUnit());
        final Acceleration a18 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final Acceleration a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, a19.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a19.getUnit());
        final Acceleration a20 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final Acceleration a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, a21.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a21.getUnit());
        final Acceleration a22 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final AccelerationTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad5.getUnit());
        final AccelerationTriad triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final Acceleration a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, a23.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a23.getUnit());
        final Acceleration a24 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final Acceleration a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, a25.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a25.getUnit());
        final Acceleration a26 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final Acceleration a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, a27.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a27.getUnit());
        final Acceleration a28 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final AccelerationTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad7.getUnit());
        final AccelerationTriad triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testConstructor2() {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                detector.getWindowSize());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                detector.getInitialStaticSamples());
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertSame(this, detector.getListener());
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final Acceleration a3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final Acceleration a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final Acceleration a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final Acceleration a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final Acceleration a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final Acceleration a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final Acceleration a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final Acceleration a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final Acceleration a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final Acceleration a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final Acceleration a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, a13.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a13.getUnit());
        final Acceleration a14 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final Acceleration a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, a15.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a15.getUnit());
        final Acceleration a16 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final AccelerationTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad3.getUnit());
        final AccelerationTriad triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final Acceleration a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, a17.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a17.getUnit());
        final Acceleration a18 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final Acceleration a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, a19.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a19.getUnit());
        final Acceleration a20 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final Acceleration a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, a21.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a21.getUnit());
        final Acceleration a22 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final AccelerationTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad5.getUnit());
        final AccelerationTriad triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final Acceleration a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, a23.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a23.getUnit());
        final Acceleration a24 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final Acceleration a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, a25.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a25.getUnit());
        final Acceleration a26 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final Acceleration a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, a27.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a27.getUnit());
        final Acceleration a28 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final AccelerationTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad7.getUnit());
        final AccelerationTriad triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
    public void testGetSetInitialStaticSamples() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
    public void testGetSetThresholdFactor() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
    public void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

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
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final Acceleration a1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());

        // set new value
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final Acceleration a3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration a4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(this, detector.getListener());
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);

        // set new value
        final double timeInterval = 2 * TIME_INTERVAL_SECONDS;
        detector.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, detector.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setTimeInterval(-1.0));
    }

    @Test
    public void testGetSetTimeInterval2() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        final Time timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        detector.setTimeInterval(timeInterval2);

        final Time timeInterval3 = detector.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);

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

        final AccelerationTriad lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                    measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
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
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final Acceleration stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final Acceleration stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(accelNoiseStd, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final Acceleration stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        for (int i = 0; i < periodLength; i++) {
            final double newRoll = oldRoll + deltaRoll;
            final double newPitch = oldPitch + deltaPitch;
            final double newYaw = oldYaw + deltaYaw;
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
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
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);

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

        final AccelerationTriad lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        final Acceleration aX = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration aY = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration aZ = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
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
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final Acceleration stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final Acceleration stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(accelNoiseStd, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final Acceleration stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        for (int i = 0; i < periodLength; i++) {
            final double newRoll = oldRoll + deltaRoll;
            final double newPitch = oldPitch + deltaPitch;
            final double newYaw = oldYaw + deltaYaw;
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
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
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);

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

        final AccelerationTriad lastStaticTriad = trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
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
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(accelNoiseStd, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final Acceleration stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(accelNoiseStd, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final Acceleration stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(stdZ1.getValue().doubleValue(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final Acceleration stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                    measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        for (int i = 0; i < periodLength; i++) {
            final double newRoll = oldRoll + deltaRoll;
            final double newPitch = oldPitch + deltaPitch;
            final double newYaw = oldYaw + deltaYaw;
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                    measuredKinematics);
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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
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
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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
    public void testProcessWithExcessiveOverallNoiseDuringInitialization() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mError);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED,
                mErrorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Test
    public void testProcessWithSuddenMotionDuringInitialization() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);

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
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(AccelerationTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some samples while keeping accelerometer body static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());

        // then add samples with motion
        final double deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        for (int i = 0; i < periodLength; i++) {
            final double newRoll = oldRoll + deltaRoll;
            final double newPitch = oldPitch + deltaPitch;
            final double newYaw = oldYaw + deltaYaw;
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

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

            if (mError != 0) {
                break;
            }
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mError);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(AccelerationTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,
                mErrorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final AccelerationTriadStaticIntervalDetector detector) {
        mInitializationStarted++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final AccelerationTriadStaticIntervalDetector detector, final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
    }

    @Override
    public void onError(
            final AccelerationTriadStaticIntervalDetector detector, final double accumulatedNoiseLevel,
            final double instantaneousNoiseLevel, final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        mErrorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        mErrorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        mErrorReason = reason;
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
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final Acceleration a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final Acceleration a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final Acceleration a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final Acceleration a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final Acceleration a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final AccelerationTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final AccelerationTriad avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final Acceleration a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final Acceleration a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final Acceleration a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final Acceleration a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final Acceleration a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final Acceleration a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final AccelerationTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final AccelerationTriad stdTriad2 = new AccelerationTriad();
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
        mDynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final Acceleration ax1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(accumulatedAvgX, ax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, ax1.getUnit());
        final Acceleration ax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(ax2);
        assertEquals(ax1, ax2);

        final Acceleration ay1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(accumulatedAvgY, ay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, ay1.getUnit());
        final Acceleration ay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(ay2);
        assertEquals(ay1, ay2);

        final Acceleration az1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(accumulatedAvgZ, az1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, az1.getUnit());
        final Acceleration az2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(az2);
        assertEquals(az1, az2);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(accumulatedAvgX, triad1.getValueX(), 0.0);
        assertEquals(accumulatedAvgY, triad1.getValueY(), 0.0);
        assertEquals(accumulatedAvgZ, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final Acceleration a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, a1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final Acceleration a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, a3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a3.getUnit());
        final Acceleration a4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final Acceleration a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, a5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a5.getUnit());
        final Acceleration a6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final AccelerationTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final AccelerationTriad avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final Acceleration a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, a7.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a7.getUnit());
        final Acceleration a8 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final Acceleration a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, a9.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a9.getUnit());
        final Acceleration a10 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final Acceleration a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, a11.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a11.getUnit());
        final Acceleration a12 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final AccelerationTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final AccelerationTriad stdTriad2 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final AccelerationTriadStaticIntervalDetector detector) {
        mReset++;
        checkLocked(detector);
    }

    private void reset() {
        mInitializationStarted = 0;
        mInitializationCompleted = 0;
        mError = 0;
        mStaticIntervalDetected = 0;
        mDynamicIntervalDetected = 0;
        mReset = 0;
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
        final Time timeInterval = new Time(1.0, TimeUnit.DAY);
        assertThrows(LockedException.class, () -> detector.setTimeInterval(timeInterval));
        final AccelerationTriad triad = new AccelerationTriad();
        assertThrows(LockedException.class, () -> detector.process(triad));
        final Acceleration a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
}
