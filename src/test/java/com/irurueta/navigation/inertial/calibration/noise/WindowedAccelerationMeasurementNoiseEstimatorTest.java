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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class WindowedAccelerationMeasurementNoiseEstimatorTest implements 
        WindowedAccelerationMeasurementNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int start;
    private int measurementAdded;
    private int windowFilled;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();

        // check default values
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final var avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final var avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());

        // set a new value
        estimator.setWindowSize(3);

        // check
        assertEquals(3, estimator.getWindowSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setWindowSize(1));
        assertThrows(IllegalArgumentException.class, () -> estimator.setWindowSize(2));
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        // set a new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var time2 = new Time(500, TimeUnit.MILLISECOND);
        estimator.setTimeInterval(time2);

        // check
        final var time3 = estimator.getTimeIntervalAsTime();
        final var time4 = new Time(0.0, TimeUnit.SECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertTrue(time2.equals(time3, ABSOLUTE_ERROR));
        assertTrue(time2.equals(time4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddMeasurementAndProcessAndThenReset1() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPsd();
        final var gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);
        
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = 0.0;
        final var omegaY = 0.0;
        final var omegaZ = 0.0;

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var measurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        double avg = 0.0;
        final var measurements = new ArrayList<Acceleration>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastWindowedMeasurement());
                assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurementAndProcess(value);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());

            avg += value;
        }

        avg /= windowSize;

        var v = 0.0;
        for (var i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final var diff = value - avg;

            v += diff * diff;
        }

        v /= (windowSize - 1);

        final var std = Math.sqrt(v);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var a1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, start);
        assertEquals(measurementAdded, windowSize);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getSpecificForceNormAsAcceleration(measurement);

        measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

        value = measurement.getValue().doubleValue();
        estimator.addMeasurementAndProcess(value);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(measurements.size(), windowSize + 1);
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(1, start);
        assertEquals(windowSize + 1, measurementAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddMeasurementAndProcessAndThenReset2() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPsd();
        final var gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = 0.0;
        final var omegaY = 0.0;
        final var omegaZ = 0.0;

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var measurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        var avg = 0.0;
        final var measurements = new ArrayList<Acceleration>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurementAndProcess(measurement);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());

            avg += value;
        }

        avg /= windowSize;

        double v = 0.0;
        for (var i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final var diff = value - avg;

            v += diff * diff;
        }

        v /= (windowSize - 1);

        final var std = Math.sqrt(v);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var a1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, start);
        assertEquals(windowSize, measurementAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getSpecificForceNormAsAcceleration(measurement);

        measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

        estimator.addMeasurementAndProcess(measurement);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, measurements.size());
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(1, start);
        assertEquals(windowSize + 1, measurementAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddMeasurement1() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPsd();
        final var gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = 0.0;
        final var omegaY = 0.0;
        final var omegaZ = 0.0;

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var measurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        final var measurements = new ArrayList<Acceleration>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastWindowedMeasurement());
                assertEquals(estimator.getLastWindowedMeasurementValue(), lastMeasurement.getValue().doubleValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, start);
        assertEquals(windowSize, measurementAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        assertEquals(windowSize, measurements.size());
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(measurements.get(0), firstMeasurement);
        assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(), 0.0);
        assertEquals(measurements.get(windowSize - 1), estimator.getLastWindowedMeasurement());
        assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
        assertEquals(lastMeasurement, measurements.get(windowSize - 1));
        assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(), 0.0);
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    void testAddMeasurement2() throws WrongSizeException, LockedException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPsd();
        final var gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var omegaX = 0.0;
        final var omegaY = 0.0;
        final var omegaZ = 0.0;

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var measurement = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var measurements = new ArrayList<Acceleration>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

            estimator.addMeasurement(measurement);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, start);
        assertEquals(windowSize, measurementAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        assertEquals(windowSize, measurements.size());
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(firstMeasurement, measurements.get(0));
        assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                0.0);
        assertEquals(measurements.get(windowSize - 1), estimator.getLastWindowedMeasurement());
        assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
        assertEquals(lastMeasurement, measurements.get(windowSize - 1));
        assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(), 0.0);
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onMeasurementAdded(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        measurementAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        windowFilled++;
    }

    @Override
    public void onReset(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        measurementAdded = 0;
        windowFilled = 0;
        reset = 0;
    }

    private void checkLocked(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setWindowSize(3));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(0.0));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(a));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(a));
        assertThrows(LockedException.class, estimator::reset);
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

    private static Matrix generateMa() throws WrongSizeException {
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

    private static double getAccelNoiseRootPsd() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPsd() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
