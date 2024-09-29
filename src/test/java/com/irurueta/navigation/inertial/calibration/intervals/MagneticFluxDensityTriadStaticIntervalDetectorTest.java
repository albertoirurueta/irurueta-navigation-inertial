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
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

import static org.junit.Assert.*;

public class MagneticFluxDensityTriadStaticIntervalDetectorTest implements
        MagneticFluxDensityTriadStaticIntervalDetectorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

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

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-8;

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                detector.getInitialStaticSamples());
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
                detector.getThresholdFactor(), 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertNull(detector.getListener());
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final MagneticFluxDensity b3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getThresholdAsMeasurement(b4);
        assertEquals(b3, b4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final MagneticFluxDensity b5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, b5.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b5.getUnit());
        final MagneticFluxDensity b6 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgXAsMeasurement(b6);
        assertEquals(b5, b6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final MagneticFluxDensity b7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, b7.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b7.getUnit());
        final MagneticFluxDensity b8 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgYAsMeasurement(b8);
        assertEquals(b7, b8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final MagneticFluxDensity b9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, b9.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b9.getUnit());
        final MagneticFluxDensity b10 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgZAsMeasurement(b10);
        assertEquals(b9, b10);

        final MagneticFluxDensityTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final MagneticFluxDensity b11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, b11.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b11.getUnit());
        final MagneticFluxDensity b12 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdXAsMeasurement(b12);
        assertEquals(b11, b12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final MagneticFluxDensity b13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, b13.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b13.getUnit());
        final MagneticFluxDensity b14 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdYAsMeasurement(b14);
        assertEquals(b13, b14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final MagneticFluxDensity b15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, b15.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b15.getUnit());
        final MagneticFluxDensity b16 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdZAsMeasurement(b16);
        assertEquals(b15, b16);

        final MagneticFluxDensityTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad3.getUnit());
        final MagneticFluxDensityTriad triad4 = new MagneticFluxDensityTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final MagneticFluxDensity b17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, b17.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b17.getUnit());
        final MagneticFluxDensity b18 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgXAsMeasurement(b18);
        assertEquals(b17, b18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final MagneticFluxDensity b19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, b19.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b19.getUnit());
        final MagneticFluxDensity b20 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgYAsMeasurement(b20);
        assertEquals(b19, b20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final MagneticFluxDensity b21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, b21.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b21.getUnit());
        final MagneticFluxDensity b22 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgZAsMeasurement(b22);
        assertEquals(b21, b22);

        final MagneticFluxDensityTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad5.getUnit());
        final MagneticFluxDensityTriad triad6 = new MagneticFluxDensityTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final MagneticFluxDensity b23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, b23.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b23.getUnit());
        final MagneticFluxDensity b24 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdXAsMeasurement(b24);
        assertEquals(b23, b24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final MagneticFluxDensity b25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, b25.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b25.getUnit());
        final MagneticFluxDensity b26 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdYAsMeasurement(b26);
        assertEquals(b25, b26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final MagneticFluxDensity b27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, b27.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b27.getUnit());
        final MagneticFluxDensity b28 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdZAsMeasurement(b28);
        assertEquals(b27, b28);

        final MagneticFluxDensityTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad7.getUnit());
        final MagneticFluxDensityTriad triad8 = new MagneticFluxDensityTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testConstructor2() {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                detector.getInitialStaticSamples());
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
                detector.getThresholdFactor(), 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertSame(this, detector.getListener());
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final MagneticFluxDensity b3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b3.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b4);
        assertEquals(b3, b4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final MagneticFluxDensity b5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, b5.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b5.getUnit());
        final MagneticFluxDensity b6 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgXAsMeasurement(b6);
        assertEquals(b5, b6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final MagneticFluxDensity b7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, b7.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b7.getUnit());
        final MagneticFluxDensity b8 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgYAsMeasurement(b8);
        assertEquals(b7, b8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final MagneticFluxDensity b9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, b9.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b9.getUnit());
        final MagneticFluxDensity b10 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedAvgZAsMeasurement(b10);
        assertEquals(b9, b10);

        final MagneticFluxDensityTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final MagneticFluxDensity b11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, b11.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b11.getUnit());
        final MagneticFluxDensity b12 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdXAsMeasurement(b12);
        assertEquals(b11, b12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final MagneticFluxDensity b13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, b13.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b13.getUnit());
        final MagneticFluxDensity b14 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdYAsMeasurement(b14);
        assertEquals(b13, b14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final MagneticFluxDensity b15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, b15.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b15.getUnit());
        final MagneticFluxDensity b16 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdZAsMeasurement(b16);
        assertEquals(b15, b16);

        final MagneticFluxDensityTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad3.getUnit());
        final MagneticFluxDensityTriad triad4 = new MagneticFluxDensityTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final MagneticFluxDensity b17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, b17.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b17.getUnit());
        final MagneticFluxDensity b18 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgXAsMeasurement(b18);
        assertEquals(b17, b18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final MagneticFluxDensity b19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, b19.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b19.getUnit());
        final MagneticFluxDensity b20 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgYAsMeasurement(b20);
        assertEquals(b19, b20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final MagneticFluxDensity b21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, b21.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b21.getUnit());
        final MagneticFluxDensity b22 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgZAsMeasurement(b22);
        assertEquals(b21, b22);

        final MagneticFluxDensityTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad5.getUnit());
        final MagneticFluxDensityTriad triad6 = new MagneticFluxDensityTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final MagneticFluxDensity b23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, b23.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b23.getUnit());
        final MagneticFluxDensity b24 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdXAsMeasurement(b24);
        assertEquals(b23, b24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final MagneticFluxDensity b25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, b25.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b25.getUnit());
        final MagneticFluxDensity b26 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdYAsMeasurement(b26);
        assertEquals(b25, b26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final MagneticFluxDensity b27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, b27.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b27.getUnit());
        final MagneticFluxDensity b28 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdZAsMeasurement(b28);
        assertEquals(b27, b28);

        final MagneticFluxDensityTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad7.getUnit());
        final MagneticFluxDensityTriad triad8 = new MagneticFluxDensityTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR,
                detector.getThresholdFactor(), 0.0);

        // set new value
        detector.setThresholdFactor(1.0);

        // check
        assertEquals(1.0, detector.getThresholdFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> detector.setThresholdFactor(0.0));
    }

    @Test
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
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

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.setBaseNoiseLevelAbsoluteThreshold(b2);

        // check
        final MagneticFluxDensity b3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(b4);
        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(this, detector.getListener());
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

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
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, null,
                timestamp, nedPosition, cnb);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED,
                detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()), SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(), Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(), detector.getBaseNoiseLevel() * detector.getThresholdFactor(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
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

        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdZ(), SMALL_ABSOLUTE_ERROR);
        final MagneticFluxDensity stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final MagneticFluxDensity stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final MagneticFluxDensity stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

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
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(MagneticFluxDensityUnit.TESLA, lastStaticTriad.getUnit());
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
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, null,
                timestamp, nedPosition, cnb);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final MagneticFluxDensity bX = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bY = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bZ = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED,
                detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(), Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
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

        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdZ(), SMALL_ABSOLUTE_ERROR);
        final MagneticFluxDensity stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final MagneticFluxDensity stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final MagneticFluxDensity stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));

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
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(MagneticFluxDensityUnit.TESLA, lastStaticTriad.getUnit());
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
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, null,
                timestamp, nedPosition, cnb);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED,
                detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
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

        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MAGNETOMETER_NOISE_STD, detector.getAccumulatedStdZ(), SMALL_ABSOLUTE_ERROR);
        final MagneticFluxDensity stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdX1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final MagneticFluxDensity stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdY1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final MagneticFluxDensity stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(MAGNETOMETER_NOISE_STD, stdZ1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

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
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(MagneticFluxDensityUnit.TESLA, lastStaticTriad.getUnit());
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
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(0, detector.getProcessedSamples());
    }

    @Test
    public void testProcessWithExcessiveOverallNoiseDuringInitialization() throws IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mError);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED,
                mErrorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Test
    public void testProcessWithSuddenMotionDuringInitialization() throws InvalidSourceAndDestinationFrameTypeException,
            IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(new Random(), 0.0,
                MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        reset();
        assertEquals(0, mInitializationStarted);
        assertEquals(0, mInitializationCompleted);
        assertEquals(0, mError);
        assertEquals(0, mStaticIntervalDetected);
        assertEquals(0, mDynamicIntervalDetected);
        assertEquals(0, mReset);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some samples while keeping magnetometer body static
        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());

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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition, cnb);

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
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,
                mErrorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
        mInitializationStarted++;
        checkLocked(detector);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final MagneticFluxDensityTriadStaticIntervalDetector detector, final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detector);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED,
                detector.getStatus());
    }

    @Override
    public void onError(final MagneticFluxDensityTriadStaticIntervalDetector detector,
                        final double accumulatedNoiseLevel,
                        final double instantaneousNoiseLevel,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        mErrorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        mErrorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        mErrorReason = reason;
        checkLocked(detector);
    }

    @Override
    public void onStaticIntervalDetected(final MagneticFluxDensityTriadStaticIntervalDetector detector,
                                         final double instantaneousAvgX,
                                         final double instantaneousAvgY,
                                         final double instantaneousAvgZ,
                                         final double instantaneousStdX,
                                         final double instantaneousStdY,
                                         final double instantaneousStdZ) {
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());

        assertEquals(detector.getInstantaneousAvgX(), instantaneousAvgX, 0.0);
        final MagneticFluxDensity b1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), instantaneousAvgX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgXAsMeasurement(b2);
        assertEquals(b1, b2);

        assertEquals(detector.getInstantaneousAvgY(), instantaneousAvgY, 0.0);
        final MagneticFluxDensity b3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(b3.getValue().doubleValue(), instantaneousAvgY, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgYAsMeasurement(b4);
        assertEquals(b3, b4);

        assertEquals(detector.getInstantaneousAvgZ(), instantaneousAvgZ, 0.0);
        final MagneticFluxDensity b5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(b5.getValue().doubleValue(), instantaneousAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b5.getUnit());
        final MagneticFluxDensity b6 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgZAsMeasurement(b6);
        assertEquals(b5, b6);

        final MagneticFluxDensityTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(avgTriad1.getValueX(), instantaneousAvgX, 0.0);
        assertEquals(avgTriad1.getValueY(), instantaneousAvgY, 0.0);
        assertEquals(avgTriad1.getValueZ(), instantaneousAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgTriad1.getUnit());
        final MagneticFluxDensityTriad avgTriad2 = new MagneticFluxDensityTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(detector.getInstantaneousStdX(), instantaneousStdX, 0.0);
        final MagneticFluxDensity b7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(b7.getValue().doubleValue(), instantaneousStdX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b7.getUnit());
        final MagneticFluxDensity b8 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdXAsMeasurement(b8);
        assertEquals(b7, b8);

        assertEquals(detector.getInstantaneousStdY(), instantaneousStdY, 0.0);
        final MagneticFluxDensity b9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(b9.getValue().doubleValue(), instantaneousStdY, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b9.getUnit());
        final MagneticFluxDensity b10 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdYAsMeasurement(b10);
        assertEquals(b9, b10);

        assertEquals(detector.getInstantaneousStdZ(), instantaneousStdZ, 0.0);
        final MagneticFluxDensity b11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(b11.getValue().doubleValue(), instantaneousStdZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b11.getUnit());
        final MagneticFluxDensity b12 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdZAsMeasurement(b12);
        assertEquals(b11, b12);

        final MagneticFluxDensityTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(stdTriad1.getValueX(), instantaneousStdX, 0.0);
        assertEquals(stdTriad1.getValueY(), instantaneousStdY, 0.0);
        assertEquals(stdTriad1.getValueZ(), instantaneousStdZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onDynamicIntervalDetected(final MagneticFluxDensityTriadStaticIntervalDetector detector,
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
        assertEquals(MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final MagneticFluxDensity bx1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(bx1.getValue().doubleValue(), accumulatedAvgX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAccumulatedAvgXAsMeasurement(bx2);
        assertEquals(bx1, bx2);

        final MagneticFluxDensity by1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(by1.getValue().doubleValue(), accumulatedAvgY, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        final MagneticFluxDensity by2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAccumulatedAvgYAsMeasurement(by2);
        assertEquals(by1, by2);

        final MagneticFluxDensity bz1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(bz1.getValue().doubleValue(), accumulatedAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAccumulatedAvgZAsMeasurement(bz2);
        assertEquals(bz1, bz2);

        final MagneticFluxDensityTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(triad1.getValueX(), accumulatedAvgX, 0.0);
        assertEquals(triad1.getValueY(), accumulatedAvgY, 0.0);
        assertEquals(triad1.getValueZ(), accumulatedAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());

        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(detector.getInstantaneousAvgX(), instantaneousAvgX, 0.0);
        final MagneticFluxDensity b1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), instantaneousAvgX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgXAsMeasurement(b2);
        assertEquals(b1, b2);

        assertEquals(detector.getInstantaneousAvgY(), instantaneousAvgY, 0.0);
        final MagneticFluxDensity b3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(b3.getValue().doubleValue(), instantaneousAvgY, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b3.getUnit());
        final MagneticFluxDensity b4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgYAsMeasurement(b4);
        assertEquals(b3, b4);

        assertEquals(detector.getInstantaneousAvgZ(), instantaneousAvgZ, 0.0);
        final MagneticFluxDensity b5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(b5.getValue().doubleValue(), instantaneousAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b5.getUnit());
        final MagneticFluxDensity b6 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousAvgZAsMeasurement(b6);
        assertEquals(b5, b6);

        final MagneticFluxDensityTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(avgTriad1.getValueX(), instantaneousAvgX, 0.0);
        assertEquals(avgTriad1.getValueY(), instantaneousAvgY, 0.0);
        assertEquals(avgTriad1.getValueZ(), instantaneousAvgZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgTriad1.getUnit());
        final MagneticFluxDensityTriad avgTriad2 = new MagneticFluxDensityTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(detector.getInstantaneousStdX(), instantaneousStdX, 0.0);
        final MagneticFluxDensity b7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(b7.getValue().doubleValue(), instantaneousStdX, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b7.getUnit());
        final MagneticFluxDensity b8 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdXAsMeasurement(b8);
        assertEquals(b7, b8);

        assertEquals(detector.getInstantaneousStdY(), instantaneousStdY, 0.0);
        final MagneticFluxDensity b9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(b9.getValue().doubleValue(), instantaneousStdY, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b9.getUnit());
        final MagneticFluxDensity b10 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdYAsMeasurement(b10);
        assertEquals(b9, b10);

        assertEquals(detector.getInstantaneousStdZ(), instantaneousStdZ, 0.0);
        final MagneticFluxDensity b11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(b11.getValue().doubleValue(), instantaneousStdZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b11.getUnit());
        final MagneticFluxDensity b12 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        detector.getInstantaneousStdZAsMeasurement(b12);
        assertEquals(b11, b12);

        final MagneticFluxDensityTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(stdTriad1.getValueX(), instantaneousStdX, 0.0);
        assertEquals(stdTriad1.getValueY(), instantaneousStdY, 0.0);
        assertEquals(stdTriad1.getValueZ(), instantaneousStdZ, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
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

    private void checkLocked(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
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
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        assertThrows(LockedException.class, () -> detector.process(triad));
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(LockedException.class, () -> detector.process(b, b, b));
        assertThrows(LockedException.class, () -> detector.process(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, detector::reset);
    }

    private static MagneticFluxDensityTriad generateTriad(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final Date timestamp, final NEDPosition position,
            final CoordinateTransformation cnb) {

        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(position, timestamp);

        final BodyMagneticFluxDensity truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final BodyMagneticFluxDensity measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron,
                softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic.getCoordinatesAsTriad();
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
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

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
