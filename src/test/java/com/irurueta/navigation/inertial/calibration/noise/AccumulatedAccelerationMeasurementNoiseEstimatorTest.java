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

class AccumulatedAccelerationMeasurementNoiseEstimatorTest implements
        AccumulatedAccelerationMeasurementNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int N_SAMPLES = 1000;

    private int start;
    private int measurementAdded;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default values
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator();

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

        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final var triad = new AccelerationTriad();
        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        double value;
        var avg = 0.0;
        var v = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastMeasurement());
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            value = triad.getNorm();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final var diff = value - avg;

            final var diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, measurementAdded);
        assertEquals(0, reset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final var avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final var std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
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

        final var estimator = new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final var triad = new AccelerationTriad();
        final var kinematics = new BodyKinematics();
        final var timeInterval = estimator.getTimeInterval();
        final var lastMeasurement = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        double value;
        var avg = 0.0;
        var v = 0.0;
        final var random = new Random();
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastMeasurement());
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            value = triad.getNorm();

            estimator.addMeasurement(triad.getMeasurementNorm());

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final var diff = value - avg;

            final var diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, measurementAdded);
        assertEquals(0, reset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final var avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final var std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final var std2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onMeasurementAdded(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        measurementAdded++;
    }

    @Override
    public void onReset(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        measurementAdded = 0;
        reset = 0;
    }

    private void checkLocked(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        final var a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
