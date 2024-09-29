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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AngularSpeedTriadStaticIntervalDetectorTest implements AngularSpeedTriadStaticIntervalDetectorListener {

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
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final AngularSpeed w3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final AngularSpeed w4 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getThresholdAsMeasurement(w4);
        assertEquals(w3, w4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final AngularSpeed w5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final AngularSpeed w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(w6);
        assertEquals(w5, w6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final AngularSpeed w7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final AngularSpeed w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final AngularSpeed w9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final AngularSpeed w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(w10);
        assertEquals(w9, w10);

        final AngularSpeedTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final AngularSpeed w11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final AngularSpeed w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(w12);
        assertEquals(w11, w12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final AngularSpeed w13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, w13.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w13.getUnit());
        final AngularSpeed w14 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(w14);
        assertEquals(w13, w14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final AngularSpeed w15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, w15.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w15.getUnit());
        final AngularSpeed w16 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(w16);
        assertEquals(w15, w16);

        final AngularSpeedTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final AngularSpeed w17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, w17.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w17.getUnit());
        final AngularSpeed w18 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w18);
        assertEquals(w17, w18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final AngularSpeed w19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, w19.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w19.getUnit());
        final AngularSpeed w20 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w20);
        assertEquals(w19, w20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final AngularSpeed w21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, w21.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w21.getUnit());
        final AngularSpeed w22 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w22);
        assertEquals(w21, w22);

        final AngularSpeedTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad5.getUnit());
        final AngularSpeedTriad triad6 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final AngularSpeed w23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, w23.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w23.getUnit());
        final AngularSpeed w24 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w24);
        assertEquals(w23, w24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final AngularSpeed w25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, w25.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w25.getUnit());
        final AngularSpeed w26 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w26);
        assertEquals(w25, w26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final AngularSpeed w27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, w27.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w27.getUnit());
        final AngularSpeed w28 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w28);
        assertEquals(w27, w28);

        final AngularSpeedTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad7.getUnit());
        final AngularSpeedTriad triad8 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testConstructor2() {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(
                this);

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
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        final AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(0.0, detector.getThreshold(), 0.0);
        final AngularSpeed w3 = detector.getThresholdAsMeasurement();
        assertEquals(0.0, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final AngularSpeed w4 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getThresholdAsMeasurement(w4);
        assertEquals(w3, w4);
        assertFalse(detector.isRunning());
        assertEquals(0, detector.getProcessedSamples());

        assertEquals(0.0, detector.getAccumulatedAvgX(), 0.0);
        final AngularSpeed w5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(0.0, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final AngularSpeed w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(w6);
        assertEquals(w5, w6);

        assertEquals(0.0, detector.getAccumulatedAvgY(), 0.0);
        final AngularSpeed w7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(0.0, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final AngularSpeed w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(0.0, detector.getAccumulatedAvgZ(), 0.0);
        final AngularSpeed w9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(0.0, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final AngularSpeed w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(w10);
        assertEquals(w9, w10);

        final AngularSpeedTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, detector.getAccumulatedStdX(), 0.0);
        final AngularSpeed w11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(0.0, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final AngularSpeed w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(w12);
        assertEquals(w11, w12);

        assertEquals(0.0, detector.getAccumulatedStdY(), 0.0);
        final AngularSpeed w13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(0.0, w13.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w13.getUnit());
        final AngularSpeed w14 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(w14);
        assertEquals(w13, w14);

        assertEquals(0.0, detector.getAccumulatedStdZ(), 0.0);
        final AngularSpeed w15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(0.0, w15.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w15.getUnit());
        final AngularSpeed w16 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(w16);
        assertEquals(w15, w16);

        final AngularSpeedTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, detector.getInstantaneousAvgX(), 0.0);
        final AngularSpeed w17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(0.0, w17.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w17.getUnit());
        final AngularSpeed w18 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w18);
        assertEquals(w17, w18);

        assertEquals(0.0, detector.getInstantaneousAvgY(), 0.0);
        final AngularSpeed w19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(0.0, w19.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w19.getUnit());
        final AngularSpeed w20 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w20);
        assertEquals(w19, w20);

        assertEquals(0.0, detector.getInstantaneousAvgZ(), 0.0);
        final AngularSpeed w21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(0.0, w21.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w21.getUnit());
        final AngularSpeed w22 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w22);
        assertEquals(w21, w22);

        final AngularSpeedTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(0.0, triad5.getValueX(), 0.0);
        assertEquals(0.0, triad5.getValueY(), 0.0);
        assertEquals(0.0, triad5.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad5.getUnit());
        final AngularSpeedTriad triad6 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(0.0, detector.getInstantaneousStdX(), 0.0);
        final AngularSpeed w23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(0.0, w23.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w23.getUnit());
        final AngularSpeed w24 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w24);
        assertEquals(w23, w24);

        assertEquals(0.0, detector.getInstantaneousStdY(), 0.0);
        final AngularSpeed w25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(0.0, w25.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w25.getUnit());
        final AngularSpeed w26 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w26);
        assertEquals(w25, w26);

        assertEquals(0.0, detector.getInstantaneousStdZ(), 0.0);
        final AngularSpeed w27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(0.0, w27.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w27.getUnit());
        final AngularSpeed w28 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w28);
        assertEquals(w27, w28);

        final AngularSpeedTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(0.0, triad7.getValueX(), 0.0);
        assertEquals(0.0, triad7.getValueY(), 0.0);
        assertEquals(0.0, triad7.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad7.getUnit());
        final AngularSpeedTriad triad8 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
    public void testGetSetInitialStaticSamples() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
    public void testGetSetThresholdFactor() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
    public void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                detector.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        final AngularSpeed w1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(AngularSpeedTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());

        // set new value
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(w2);

        // check
        final AngularSpeed w3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final AngularSpeed w4 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(w4);
        assertEquals(w2, w3);
        assertEquals(w2, w4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(this, detector.getListener());
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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
        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector();

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

        final double gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(
                this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final AngularSpeed stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final AngularSpeed stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final AngularSpeed stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + periodLength, detector.getProcessedSamples());

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random,
                    measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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

        final double gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final AngularSpeed wX = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed wY = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed wZ = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);

        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final AngularSpeed stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final AngularSpeed stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final AngularSpeed stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);
            triad.getMeasurementX(wX);
            triad.getMeasurementY(wY);
            triad.getMeasurementZ(wZ);

            assertTrue(detector.process(wX, wY, wZ));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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

        final double gyroNoiseStd = gyroNoiseRootPSD / Math.sqrt(TIME_INTERVAL_SECONDS);

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
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

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

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(
                this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());
        assertEquals(0.0, detector.getBaseNoiseLevel(), 0.0);
        AngularSpeed w1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mInitializationCompleted);
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
        assertEquals(detector.getBaseNoiseLevel() * detector.getThresholdFactor(), detector.getThreshold(), 0.0);
        w1 = detector.getThresholdAsMeasurement();
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        detector.getThresholdAsMeasurement(w2);
        assertEquals(w1, w2);
        assertEquals(initialStaticSamples, detector.getProcessedSamples());

        assertEquals(gyroNoiseStd, detector.getAccumulatedStdX(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdY(), ABSOLUTE_ERROR);
        assertEquals(gyroNoiseStd, detector.getAccumulatedStdZ(), ABSOLUTE_ERROR);
        final AngularSpeed stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(gyroNoiseStd, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final AngularSpeed stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(gyroNoiseStd, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final AngularSpeed stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(gyroNoiseStd, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
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

        assertEquals(1, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 2L * periodLength, detector.getProcessedSamples());

        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(2, mStaticIntervalDetected);
        assertEquals(1, mDynamicIntervalDetected);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());
        assertEquals(initialStaticSamples + 3L * periodLength, detector.getProcessedSamples());

        // reset
        detector.reset();

        assertEquals(1, mReset);
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
    public void testProcessWithExcessiveOverallNoiseDuringInitialization()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException {
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

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(
                this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // gyroscope static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mError);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED,
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

        final AngularSpeedTriadStaticIntervalDetector detector = new AngularSpeedTriadStaticIntervalDetector(
                this);

        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.IDLE, detector.getStatus());

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some sample while keeping gyroscope body static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getAngularRateTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());

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

            if (mError != 0) {
                break;
            }
        }

        assertEquals(1, mInitializationStarted);
        assertEquals(1, mError);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.FAILED, detector.getStatus());
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED,
                mErrorReason);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final AngularSpeedTriadStaticIntervalDetector detector) {
        mInitializationStarted++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZING, detector.getStatus());
    }

    @Override
    public void onInitializationCompleted(
            final AngularSpeedTriadStaticIntervalDetector detector, final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED, detector.getStatus());
    }

    @Override
    public void onError(
            final AngularSpeedTriadStaticIntervalDetector detector, final double accumulatedNoiseLevel,
            final double instantaneousNoiseLevel, final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        mErrorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        mErrorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        mErrorReason = reason;
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
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.STATIC_INTERVAL, detector.getStatus());

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final AngularSpeed w1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final AngularSpeed w3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final AngularSpeed w4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w4);
        assertEquals(w3, w4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final AngularSpeed w5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final AngularSpeed w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w6);
        assertEquals(w5, w6);

        final AngularSpeedTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final AngularSpeedTriad avgTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final AngularSpeed w7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final AngularSpeed w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final AngularSpeed w9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final AngularSpeed w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w10);
        assertEquals(w9, w10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final AngularSpeed w11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final AngularSpeed w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w12);
        assertEquals(w11, w12);

        final AngularSpeedTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
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
        mDynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(AngularSpeedTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL, detector.getStatus());
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final AngularSpeed wx1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(accumulatedAvgX, wx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wx1.getUnit());
        final AngularSpeed wx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(wx2);
        assertEquals(wx1, wx2);

        final AngularSpeed wy1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(accumulatedAvgY, wy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wy1.getUnit());
        final AngularSpeed wy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(wy2);
        assertEquals(wy1, wy2);

        final AngularSpeed wz1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(accumulatedAvgZ, wz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, wz1.getUnit());
        final AngularSpeed wz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(wz2);
        assertEquals(wz1, wz2);

        final AngularSpeedTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(accumulatedAvgX, triad1.getValueX(), 0.0);
        assertEquals(accumulatedAvgY, triad1.getValueY(), 0.0);
        assertEquals(accumulatedAvgZ, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(instantaneousAvgX, detector.getInstantaneousAvgX(), 0.0);
        final AngularSpeed w1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(instantaneousAvgX, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(instantaneousAvgY, detector.getInstantaneousAvgY(), 0.0);
        final AngularSpeed w3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(instantaneousAvgY, w3.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w3.getUnit());
        final AngularSpeed w4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(w4);
        assertEquals(w3, w4);

        assertEquals(instantaneousAvgZ, detector.getInstantaneousAvgZ(), 0.0);
        final AngularSpeed w5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(instantaneousAvgZ, w5.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w5.getUnit());
        final AngularSpeed w6 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(w6);
        assertEquals(w5, w6);

        final AngularSpeedTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(instantaneousAvgX, avgTriad1.getValueX(), 0.0);
        assertEquals(instantaneousAvgY, avgTriad1.getValueY(), 0.0);
        assertEquals(instantaneousAvgZ, avgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final AngularSpeedTriad avgTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(instantaneousStdX, detector.getInstantaneousStdX(), 0.0);
        final AngularSpeed w7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(instantaneousStdX, w7.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w7.getUnit());
        final AngularSpeed w8 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdXAsMeasurement(w8);
        assertEquals(w7, w8);

        assertEquals(instantaneousStdY, detector.getInstantaneousStdY(), 0.0);
        final AngularSpeed w9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(instantaneousStdY, w9.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w9.getUnit());
        final AngularSpeed w10 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdYAsMeasurement(w10);
        assertEquals(w9, w10);

        assertEquals(instantaneousStdZ, detector.getInstantaneousStdZ(), 0.0);
        final AngularSpeed w11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(instantaneousStdZ, w11.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w11.getUnit());
        final AngularSpeed w12 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        detector.getInstantaneousStdZAsMeasurement(w12);
        assertEquals(w11, w12);

        final AngularSpeedTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(instantaneousStdX, stdTriad1.getValueX(), 0.0);
        assertEquals(instantaneousStdY, stdTriad1.getValueY(), 0.0);
        assertEquals(instantaneousStdZ, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final AngularSpeedTriadStaticIntervalDetector detector) {
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

    private void checkLocked(final AngularSpeedTriadStaticIntervalDetector detector) {
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
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        assertThrows(LockedException.class, () -> detector.process(triad));
        final AngularSpeed w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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
