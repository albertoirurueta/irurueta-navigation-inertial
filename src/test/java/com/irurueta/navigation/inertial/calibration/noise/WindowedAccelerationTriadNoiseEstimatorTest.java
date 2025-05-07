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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
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

class WindowedAccelerationTriadNoiseEstimatorTest implements WindowedAccelerationTriadNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int start;
    private int triadAdded;
    private int windowFilled;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new WindowedAccelerationTriadNoiseEstimator();

        // check default values
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getFirstWindowedTriad());
        assertFalse(estimator.getFirstWindowedTriad(null));
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.getLastWindowedTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgX1.getUnit());
        final var avgX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgY1.getUnit());
        final var avgY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgZ1.getUnit());
        final var avgZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final var triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testConstructor2() {
        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        // check default values
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getFirstWindowedTriad());
        assertFalse(estimator.getFirstWindowedTriad(null));
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.getLastWindowedTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgX1.getUnit());
        final var avgX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgY1.getUnit());
        final var avgY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgZ1.getUnit());
        final var avgZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final var triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var estimator = new WindowedAccelerationTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_WINDOW_SIZE, estimator.getWindowSize());

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
        final var estimator = new WindowedAccelerationTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);

        // set a new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new WindowedAccelerationTriadNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new WindowedAccelerationTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddTriadAndProcessAndThenReset1() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(firstTriad, estimator.getFirstWindowedTriad());
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(lastTriad, estimator.getLastWindowedTriad());
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriadAndProcess(triad);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx += valueX;
            avgFy += valueY;
            avgFz += valueZ;
        }

        avgFx /= windowSize;
        avgFy /= windowSize;
        avgFz /= windowSize;

        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        for (var i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final var diffX = valueX - avgFx;
            final var diffY = valueY - avgFy;
            final var diffZ = valueZ - avgFz;

            varFx += diffX * diffX;
            varFy += diffY * diffY;
            varFz += diffZ * diffZ;
        }

        varFx /= (windowSize - 1);
        varFy /= (windowSize - 1);
        varFz /= (windowSize - 1);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);

        assertEquals(avgFx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        var a1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgFx, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgFy, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgYAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgFz, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgZAsMeasurement(a2);
        assertEquals(a1, a2);

        final var triad1 = estimator.getAvgTriad();
        assertEquals(avgFx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varFx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdFx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdFy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdFz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdFx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdFy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdFz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdFx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdFy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdFz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = varFx * timeInterval;
        final var psdY = varFy * timeInterval;
        final var psdZ = varFz * timeInterval;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;
        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var normRootPsd = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getSpecificForceTriad(triad);

        triads.add(new AccelerationTriad(triad));

        estimator.addTriadAndProcess(triad);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, triads.size());
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(1, start);
        assertEquals(windowSize + 1, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddTriadAndProcessAndThenReset2() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(lastTriad, estimator.getLastWindowedTriad());
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriadAndProcess(valueX, valueY, valueZ);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgFx += valueX;
            avgFy += valueY;
            avgFz += valueZ;
        }

        avgFx /= windowSize;
        avgFy /= windowSize;
        avgFz /= windowSize;

        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        for (var i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final var diffX = valueX - avgFx;
            final var diffY = valueY - avgFy;
            final var diffZ = valueZ - avgFz;

            varFx += diffX * diffX;
            varFy += diffY * diffY;
            varFz += diffZ * diffZ;
        }

        varFx /= (windowSize - 1);
        varFy /= (windowSize - 1);
        varFz /= (windowSize - 1);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);

        assertEquals(avgFx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        var a1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgFx, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgFy, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgYAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgFz, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgZAsMeasurement(a2);
        assertEquals(a1, a2);

        final var triad1 = estimator.getAvgTriad();
        assertEquals(avgFx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varFx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdFx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdFy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdFz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdFx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdFy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdFz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdFx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdFy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdFz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = varFx * timeInterval;
        final var psdY = varFy * timeInterval;
        final var psdZ = varFz * timeInterval;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;
        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var normRootPsd = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getSpecificForceTriad(triad);

        triads.add(new AccelerationTriad(triad));

        estimator.addTriadAndProcess(triad.getValueX(), triad.getValueY(), triad.getValueZ());

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, triads.size());
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(1, start);
        assertEquals(windowSize + 1, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddTriadAndProcessAndThenReset3() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgFx = 0.0;
        var avgFy = 0.0;
        var avgFz = 0.0;
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(firstTriad, estimator.getFirstWindowedTriad());
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriadAndProcess(triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgFx += valueX;
            avgFy += valueY;
            avgFz += valueZ;
        }

        avgFx /= windowSize;
        avgFy /= windowSize;
        avgFz /= windowSize;

        var varFx = 0.0;
        var varFy = 0.0;
        var varFz = 0.0;
        for (var i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final var diffX = valueX - avgFx;
            final var diffY = valueY - avgFy;
            final var diffZ = valueZ - avgFz;

            varFx += diffX * diffX;
            varFy += diffY * diffY;
            varFz += diffZ * diffZ;
        }

        varFx /= (windowSize - 1);
        varFy /= (windowSize - 1);
        varFz /= (windowSize - 1);

        final var stdFx = Math.sqrt(varFx);
        final var stdFy = Math.sqrt(varFy);
        final var stdFz = Math.sqrt(varFz);

        assertEquals(avgFx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        var a1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgFx, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final var a2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgFy, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgYAsMeasurement(a2);
        assertEquals(a1, a2);

        a1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgFz, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        estimator.getAvgZAsMeasurement(a2);
        assertEquals(a1, a2);

        final var triad1 = estimator.getAvgTriad();
        assertEquals(avgFx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgFy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgFz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final var norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varFx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdFx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdFy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdFz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdFx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdFy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdFz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdFx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdFy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdFz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = varFx * timeInterval;
        final var psdY = varFy * timeInterval;
        final var psdZ = varFz * timeInterval;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;
        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var normRootPsd = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
        kinematics.getSpecificForceTriad(triad);

        triads.add(new AccelerationTriad(triad));

        estimator.addTriadAndProcess(triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(windowSize + 1, triads.size());
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(1, start);
        assertEquals(windowSize + 1, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddTriad1() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            estimator.addTriad(triad);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        assertEquals(windowSize, triads.size());
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    void testAddTriad2() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(firstTriad, estimator.getFirstWindowedTriad());
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(lastTriad, estimator.getLastWindowedTriad());
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            estimator.addTriad(triad.getValueX(), triad.getValueY(), triad.getValueZ());

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        assertEquals(windowSize, triads.size());
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    void testAddTriad3() throws WrongSizeException, LockedException {
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

        final var estimator = new WindowedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, windowFilled);
        assertEquals(0, reset);
        assertFalse(estimator.isWindowFilled());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final var kinematics = new BodyKinematics();
        final var firstKinematics = new BodyKinematics();
        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstTriad = new AccelerationTriad();
        final var lastTriad = new AccelerationTriad();
        final var triad = new AccelerationTriad();
        final var triads = new ArrayList<AccelerationTriad>();
        final var random = new Random();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(firstTriad, estimator.getFirstWindowedTriad());
                assertEquals(firstTriad, firstKinematics.getSpecificForceTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceTriad(triad);
            triads.add(new AccelerationTriad(triad));
            estimator.addTriad(triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
        }

        assertEquals(1, start);
        assertEquals(windowSize, triadAdded);
        assertEquals(1, windowFilled);
        assertEquals(0, reset);

        assertEquals(windowSize, triads.size());
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedAccelerationTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onTriadAdded(final WindowedAccelerationTriadNoiseEstimator estimator) {
        triadAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedAccelerationTriadNoiseEstimator estimator) {
        windowFilled++;
    }

    @Override
    public void onReset(final WindowedAccelerationTriadNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        triadAdded = 0;
        windowFilled = 0;
        reset = 0;
    }

    private void checkLocked(final WindowedAccelerationTriadNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setWindowSize(3));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addTriadAndProcess(null));
        assertThrows(LockedException.class, () -> estimator.addTriadAndProcess(0.0, 0.0, 0.0));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(LockedException.class, () -> estimator.addTriadAndProcess(a, a, a));
        assertThrows(LockedException.class, () -> estimator.addTriad(null));
        assertThrows(LockedException.class, () -> estimator.addTriad(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.addTriad(a, a, a));
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
