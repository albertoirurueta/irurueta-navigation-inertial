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

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccumulatedAccelerationTriadNoiseEstimatorTest implements AccumulatedAccelerationTriadNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int N_SAMPLES = 1000;

    private int start;
    private int triadAdded;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator();

        // check default values
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator();

        // check default value
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
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
        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddTriadAndReset1() throws LockedException, WrongSizeException {
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

        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final var triad = new AccelerationTriad();
        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgX = 0.0;
        var avgY = 0.0;
        var avgZ = 0.0;
        var varX = 0.0;
        var varY = 0.0;
        var varZ = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(lastTriad, estimator.getLastTriad());
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(valueX, valueY, valueZ);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgX1.getUnit());
        final var avgX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgY1.getUnit());
        final var avgY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgZ1.getUnit());
        final var avgZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AccelerationTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgNorm1.getUnit());
        final var avgNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

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

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testAddTriadAndReset2() throws LockedException, WrongSizeException {
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

        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final var triad = new AccelerationTriad();
        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgX = 0.0;
        double avgY = 0.0;
        double avgZ = 0.0;
        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(triad);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgX1.getUnit());
        final var avgX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgY1.getUnit());
        final var avgY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgZ1.getUnit());
        final var avgZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AccelerationTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgNorm1.getUnit());
        final var avgNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

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

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testAddTriadAndReset3() throws LockedException, WrongSizeException {
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

        final var estimator = new AccumulatedAccelerationTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final var triad = new AccelerationTriad();
        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new AccelerationTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgX = 0.0;
        double avgY = 0.0;
        double avgZ = 0.0;
        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgX1.getUnit());
        final var avgX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgY1.getUnit());
        final var avgY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgZ1.getUnit());
        final var avgZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgTriad1.getUnit());
        final var avgTriad2 = new AccelerationTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgNorm1.getUnit());
        final var avgNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdX1.getUnit());
        final var stdX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdY1.getUnit());
        final var stdY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdZ1.getUnit());
        final var stdZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdTriad1.getUnit());
        final var stdTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdNorm1.getUnit());
        final var stdNorm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStd1.getUnit());
        final var avgStd2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

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

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
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
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedAccelerationTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onTriadAdded(final AccumulatedAccelerationTriadNoiseEstimator estimator) {
        triadAdded++;
    }

    @Override
    public void onReset(final AccumulatedAccelerationTriadNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        triadAdded = 0;
        reset = 0;
    }

    private void checkLocked(final AccumulatedAccelerationTriadNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addTriad(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.addTriad(new AccelerationTriad()));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
