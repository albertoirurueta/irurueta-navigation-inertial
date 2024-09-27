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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AccumulatedAngularSpeedMeasurementNoiseEstimatorTest implements
        AccumulatedAngularSpeedMeasurementNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int N_SAMPLES = 1000;

    private int mStart;
    private int mMeasurementAdded;
    private int mReset;

    @Test
    public void testConstructor1() {
        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator();

        // check default values
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTriadAndReset1() throws LockedException, WrongSizeException {
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
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mReset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        double value;
        double avg = 0.0;
        double v = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastMeasurement());
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getAngularRateTriad(triad);

            value = triad.getNorm();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final double diff = value - avg;

            final double diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, mStart);
        assertEquals(N_SAMPLES, mMeasurementAdded);
        assertEquals(0, mReset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final AngularSpeed avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, mReset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testAddTriadAndReset2() throws LockedException, WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator =
                new AccumulatedAngularSpeedMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mMeasurementAdded);
        assertEquals(0, mReset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeed lastMeasurement = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        double value;
        double avg = 0.0;
        double v = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastMeasurement());
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics, errors, random, kinematics);
            kinematics.getAngularRateTriad(triad);

            value = triad.getNorm();

            estimator.addMeasurement(triad.getMeasurementNorm());

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final double diff = value - avg;

            final double diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, mStart);
        assertEquals(N_SAMPLES, mMeasurementAdded);
        assertEquals(0, mReset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final AngularSpeed avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeed std2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(), estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, mReset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onMeasurementAdded(final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator) {
        mMeasurementAdded++;
    }

    @Override
    public void onReset(final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mMeasurementAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final AccumulatedAngularSpeedMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        final AngularSpeed w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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

    private Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private Matrix generateGg() throws WrongSizeException {
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
