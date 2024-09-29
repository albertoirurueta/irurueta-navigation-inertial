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
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WindowedAngularSpeedMeasurementNoiseEstimatorTest implements
        WindowedAngularSpeedMeasurementNoiseEstimatorListener {

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int mStart;
    private int mMeasurementAdded;
    private int mWindowFilled;
    private int mReset;

    @Test
    public void testConstructor1() {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator();

        // check default values
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
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
        final AngularSpeed avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
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
        final AngularSpeed avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());

        // set new value
        estimator.setWindowSize(3);

        // check
        assertEquals(3, estimator.getWindowSize());

        // force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setWindowSize(1));
        assertThrows(IllegalArgumentException.class, () -> estimator.setWindowSize(2));
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set new value
        final Time time2 = new Time(500, TimeUnit.MILLISECOND);
        estimator.setTimeInterval(time2);

        // check
        final Time time3 = estimator.getTimeIntervalAsTime();
        final Time time4 = new Time(0.0, TimeUnit.SECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertTrue(time2.equals(time3, ABSOLUTE_ERROR));
        assertTrue(time2.equals(time4, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddMeasurementAndProcessAndThenReset1() throws WrongSizeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mWindowFilled);
        assertEquals(0, mReset);
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

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed firstMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed measurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        double value;
        double avg = 0.0;
        final List<AngularSpeed> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getAngularSpeedNorm());
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

            kinematics.getAngularSpeedNorm(measurement);
            measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));
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

        double v = 0.0;
        for (int i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final double diff = value - avg;

            v += diff * diff;
        }

        v /= (windowSize - 1);

        final double std = Math.sqrt(v);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final AngularSpeed w1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, mStart);
        assertEquals(windowSize, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getAngularSpeedNorm(measurement);

        measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));

        value = measurement.getValue().doubleValue();
        estimator.addMeasurementAndProcess(value);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, measurements.size());
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(1, mStart);
        assertEquals(windowSize + 1, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, mReset);

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
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurementAndProcessAndThenReset2()
            throws WrongSizeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mWindowFilled);
        assertEquals(0, mReset);
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

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed firstMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed measurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        double value;
        double avg = 0.0;
        final List<AngularSpeed> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getAngularSpeedNorm());
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

            kinematics.getAngularSpeedNorm(measurement);
            measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));
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
        for (int i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final double diff = value - avg;

            v += diff * diff;
        }

        v /= (windowSize - 1);

        final double std = Math.sqrt(v);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final AngularSpeed w1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAsMeasurement(w2);
        assertEquals(w1, w2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, mStart);
        assertEquals(windowSize, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getAngularSpeedNorm(measurement);

        measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));

        estimator.addMeasurementAndProcess(measurement);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, measurements.size());
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(1, mStart);
        assertEquals(windowSize + 1, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, mReset);

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
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurement1() throws WrongSizeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mWindowFilled);
        assertEquals(0, mReset);
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

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed firstMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed measurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        double value;
        final List<AngularSpeed> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getAngularSpeedNorm());
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

            kinematics.getAngularSpeedNorm(measurement);
            measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, mStart);
        assertEquals(windowSize, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        assertEquals(windowSize, measurements.size());
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(firstMeasurement, measurements.get(0));
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
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurement2() throws WrongSizeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final WindowedAngularSpeedMeasurementNoiseEstimator estimator =
                new WindowedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mWindowFilled);
        assertEquals(0, mReset);
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

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed firstMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed measurement = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        final List<AngularSpeed> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(estimator.getFirstWindowedMeasurement(), firstMeasurement);
                assertEquals(estimator.getFirstWindowedMeasurementValue(), firstMeasurement.getValue().doubleValue(),
                        0.0);
                assertEquals(firstMeasurement, firstKinematics.getAngularSpeedNorm());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(estimator.getLastWindowedMeasurementValue(), lastMeasurement.getValue().doubleValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularSpeedNorm(measurement);
            measurements.add(new AngularSpeed(measurement.getValue(), measurement.getUnit()));

            estimator.addMeasurement(measurement);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(i + 1, estimator.getNumberOfAddedSamples());
            assertEquals(i + 1, estimator.getNumberOfSamplesInWindow());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, mStart);
        assertEquals(windowSize, mMeasurementAdded);
        assertEquals(1, mWindowFilled);
        assertEquals(0, mReset);

        assertEquals(windowSize, measurements.size());
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(firstMeasurement, measurements.get(0));
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
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedAngularSpeedMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onMeasurementAdded(final WindowedAngularSpeedMeasurementNoiseEstimator estimator) {
        mMeasurementAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedAngularSpeedMeasurementNoiseEstimator estimator) {
        mWindowFilled++;
    }

    @Override
    public void onReset(final WindowedAngularSpeedMeasurementNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mMeasurementAdded = 0;
        mWindowFilled = 0;
        mReset = 0;
    }

    private void checkLocked(final WindowedAngularSpeedMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setWindowSize(3));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(0.0));
        final AngularSpeed w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(w));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(w));
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

    private static double getAccelNoiseRootPsd() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPsd() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
