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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AngularSpeedTriadStaticIntervalDetectorTest implements AngularSpeedTriadStaticIntervalDetectorListener {

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

    private static final double ABSOLUTE_ERROR = 1e-5;

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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                detector.getInitialStaticSamples());
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertNull(detector.getListener());
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);
        final var timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final var w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final var w3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final var w4 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getThresholdAsMeasurement(w4);
        assertEquals(w3, w4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final var w5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final var w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(w6);
        assertEquals(w5, w6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final var w7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final var w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final var w9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final var w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(w10);
        assertEquals(w9, w10);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final var triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final var w11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final var w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(w12);
        assertEquals(w11, w12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final var w13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, w13.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w13.getUnit());
        final var w14 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(w14);
        assertEquals(w13, w14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final var w15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, w15.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w15.getUnit());
        final var w16 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(w16);
        assertEquals(w15, w16);

        final var triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final var w17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, w17.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w17.getUnit());
        final var w18 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w18);
        assertEquals(w17, w18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final var w19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, w19.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w19.getUnit());
        final var w20 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w20);
        assertEquals(w19, w20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final var w21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, w21.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w21.getUnit());
        final var w22 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w22);
        assertEquals(w21, w22);

        final var triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad5.getUnit());
        final var triad6 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final var w23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, w23.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w23.getUnit());
        final var w24 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w24);
        assertEquals(w23, w24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final var w25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, w25.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w25.getUnit());
        final var w26 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w26);
        assertEquals(w25, w26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final var w27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, w27.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w27.getUnit());
        final var w28 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w28);
        assertEquals(w27, w28);

        final var triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad7.getUnit());
        final var triad8 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    void testConstructor2() {
        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, 
                detector.getInitialStaticSamples());
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
                0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                detector.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertSame(this, detector.getListener());
        assertEquals(TIME_INTERVAL_SECONDS, detector.getTimeInterval(), 0.0);
        final var timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final var w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final var w3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final var w4 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getThresholdAsMeasurement(w4);
        assertEquals(w3, w4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final var w5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final var w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(w6);
        assertEquals(w5, w6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final var w7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final var w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final var w9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final var w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(w10);
        assertEquals(w9, w10);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final var triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final var w11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final var w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(w12);
        assertEquals(w11, w12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final var w13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, w13.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w13.getUnit());
        final var w14 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(w14);
        assertEquals(w13, w14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final var w15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, w15.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w15.getUnit());
        final var w16 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(w16);
        assertEquals(w15, w16);

        final var triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final var w17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, w17.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w17.getUnit());
        final var w18 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w18);
        assertEquals(w17, w18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final var w19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, w19.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w19.getUnit());
        final var w20 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w20);
        assertEquals(w19, w20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final var w21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, w21.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w21.getUnit());
        final var w22 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w22);
        assertEquals(w21, w22);

        final var triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad5.getUnit());
        final var triad6 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final var w23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, w23.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w23.getUnit());
        final var w24 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w24);
        assertEquals(w23, w24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final var w25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, w25.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w25.getUnit());
        final var w26 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w26);
        assertEquals(w25, w26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final var w27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, w27.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w27.getUnit());
        final var w28 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w28);
        assertEquals(w27, w28);

        final var triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad7.getUnit());
        final var triad8 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, detector.getWindowSize());

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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, 
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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, detector.getThresholdFactor(),
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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final var w1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());

        // set new value
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(w2);

        // check
        final var w3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final var w4 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(w4);
        assertEquals(w2, w3);
        assertEquals(w2, w4);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(this, detector.getListener());
    }

    @Test
    void testGetSetTimeInterval1() throws LockedException {
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

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
        final var detector = new AngularSpeedTriadStaticIntervalDetector();

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

        final var gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);
        
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

        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        var w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AngularSpeedTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(w1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final var stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final var stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final var stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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
            measuredKinematics.getAngularRateTriad(triad);

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
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
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

        final var gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AngularSpeedTriad();
        final var wX = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var wY = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var wZ = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var random = new Random();

        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(w1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final var stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final var stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final var stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final var deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final var deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
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
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));

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
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
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

        final var gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        var w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0, detector.getProcessedSamples());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AngularSpeedTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, initializationCompleted);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(w1.getValue().doubleValue(), detector.getBaseNoiseLevel(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                detector.getBaseNoiseLevelRootPsd(), VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0), detector.getBaseNoiseLevelPsd(),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(),
                0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final var stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final var stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final var stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final var stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final var stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final var stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        var periodLength = 2 * detector.getWindowSize();
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, staticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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
            measuredKinematics.getAngularRateTriad(triad);

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
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (var i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(2, staticIntervalDetected);
        assertEquals(1, dynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, reset);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, detector.getBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
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

        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final var initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AngularSpeedTriad();
        final var random = new Random();
        for (var i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));

            if (error != 0) {
                break;
            }
        }

        assertEquals(1, initializationStarted);
        assertEquals(1, error);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(errorAccumulatedNoiseLevel > 0.0);
        assertTrue(errorInstantaneousNoiseLevel > 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED,
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

        final var detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final var initialStaticSamples = detector.getInitialStaticSamples();
        var periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        var halfInitialStaticSamples = initialStaticSamples / 2;

        // add some sample while keeping gyroscope body static
        final var measuredKinematics = new BodyKinematics();
        final var triad = new AngularSpeedTriad();
        final var random = new Random();
        for (var i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, initializationStarted);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());

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
            measuredKinematics.getAngularRateTriad(triad);

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
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(errorAccumulatedNoiseLevel > 0.0);
        assertTrue(errorInstantaneousNoiseLevel > 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,
                errorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final AngularSpeedTriadStaticIntervalDetector detector) {
        initializationStarted++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final AngularSpeedTriadStaticIntervalDetector detector, final double baseNoiseLevel) {
        initializationCompleted++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
    }

    @Override
    public void onError(
            final AngularSpeedTriadStaticIntervalDetector detector, final double accumulatedNoiseLevel,
            final double instantaneousNoiseLevel, final TriadStaticIntervalDetector.ErrorReason reason) {
        error++;
        errorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        errorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        errorReason = reason;
        checkLocked(detector);
    }

    @Override
    public void onStaticIntervalDetected(final AngularSpeedTriadStaticIntervalDetector detector,
                                         final double instantaneousAvgX,
                                         final double instantaneousAvgY,
                                         final double instantaneousAvgZ,
                                         final double instantaneousStdX,
                                         final double instantaneousStdY,
                                         final double instantaneousStdZ) {
        staticIntervalDetected++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final var w1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final var w3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final var w4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w4);
        assertEquals(w3, w4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final var w5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final var w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w6);
        assertEquals(w5, w6);

        final var avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final var w7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final var w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final var w9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final var w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w10);
        assertEquals(w9, w10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final var w11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final var w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w12);
        assertEquals(w11, w12);

        final var stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onDynamicIntervalDetected(
            final AngularSpeedTriadStaticIntervalDetector detector, final double instantaneousAvgX,
            final double instantaneousAvgY, final double instantaneousAvgZ, final double instantaneousStdX,
            final double instantaneousStdY, final double instantaneousStdZ, final double accumulatedAvgX,
            final double accumulatedAvgY, final double accumulatedAvgZ, final double accumulatedStdX,
            final double accumulatedStdY, final double accumulatedStdZ) {
        dynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final var wx1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(accumulatedAvgX, wx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wx1.getUnit());
        final var wx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(wx2);
        assertEquals(wx1, wx2);

        final var wy1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(accumulatedAvgY, wy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wy1.getUnit());
        final var wy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(wy2);
        assertEquals(wy1, wy2);

        final var wz1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(accumulatedAvgZ, wz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wz1.getUnit());
        final var wz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(wz2);
        assertEquals(wz1, wz2);

        final var triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(accumulatedAvgX, triad1.getValueX(), 0.0);
        assertEquals(accumulatedAvgY, triad1.getValueY(), 0.0);
        assertEquals(accumulatedAvgZ, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        final var triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final var w1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final var w2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final var w3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final var w4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w4);
        assertEquals(w3, w4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final var w5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final var w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w6);
        assertEquals(w5, w6);

        final var avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final var w7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final var w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final var w9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final var w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w10);
        assertEquals(w9, w10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final var w11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final var w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w12);
        assertEquals(w11, w12);

        final var stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final AngularSpeedTriadStaticIntervalDetector detector) {
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

    private void checkLocked(final AngularSpeedTriadStaticIntervalDetector detector) {
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
        final var triad = new AngularSpeedTriad();
        assertThrows(LockedException.class, () -> detector.process(triad));
        final var w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(LockedException.class, () -> detector.process(w, w, w));
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
