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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

import static org.junit.jupiter.api.Assertions.*;

class WindowedMagneticFluxDensityMeasurementNoiseEstimatorTest implements 
        WindowedMagneticFluxDensityMeasurementNoiseEstimatorListener {

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

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private int start;
    private int measurementAdded;
    private int windowFilled;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default values
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 
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
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());

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
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddMeasurementAndProcessAndThenReset1() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator(this);

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

        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var firstMeasurement2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var lastMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity measurement = null;
        double value;
        var avg = 0.0;
        var measurements = new ArrayList<MagneticFluxDensity>();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastWindowedMeasurement());
                assertEquals(estimator.getLastWindowedMeasurementValue(), lastMeasurement.getValue().doubleValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            measurement = generateMeasurement(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp,
                    position, cnb);

            if (i == 0) {
                firstMeasurement2.setValue(measurement.getValue());
                firstMeasurement2.setUnit(measurement.getUnit());
            }

            measurements.add(new MagneticFluxDensity(measurement.getValue(), measurement.getUnit()));
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
        assertEquals(MagneticFluxDensityUnit.TESLA, a1.getUnit());
        final var a2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

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
        measurement = generateMeasurement(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position,
                cnb);

        measurements.add(new MagneticFluxDensity(measurement.getValue(), measurement.getUnit()));

        value = measurement.getValue().doubleValue();
        estimator.addMeasurementAndProcess(value);

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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddMeasurementAndProcessAndThenReset2() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator(this);

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

        final var windowSize = estimator.getWindowSize();
        final var timeInterval = estimator.getTimeInterval();
        final var firstMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var firstMeasurement2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var lastMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity measurement = null;
        double value;
        var avg = 0.0;
        final var measurements = new ArrayList<MagneticFluxDensity>();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastWindowedMeasurement());
                assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            measurement = generateMeasurement(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp,
                    position, cnb);

            if (i == 0) {
                firstMeasurement2.setValue(measurement.getValue());
                firstMeasurement2.setUnit(measurement.getUnit());
            }

            measurements.add(new MagneticFluxDensity(measurement.getValue(), measurement.getUnit()));
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
        assertEquals(MagneticFluxDensityUnit.TESLA, a1.getUnit());
        final var a2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = v * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

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
        measurement = generateMeasurement(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position,
                cnb);

        measurements.add(new MagneticFluxDensity(measurement.getValue(), measurement.getUnit()));

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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    void testAddMeasurement1() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new WindowedMagneticFluxDensityMeasurementNoiseEstimator(this);

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

        final var windowSize = estimator.getWindowSize();
        final var firstMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var firstMeasurement2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        final var lastMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensity measurement = null;
        double value;
        final var measurements = new ArrayList<MagneticFluxDensity>();
        for (var i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(firstMeasurement, estimator.getFirstWindowedMeasurement());
                assertEquals(firstMeasurement.getValue().doubleValue(), estimator.getFirstWindowedMeasurementValue(),
                        0.0);
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(lastMeasurement, estimator.getLastWindowedMeasurement());
                assertEquals(lastMeasurement.getValue().doubleValue(), estimator.getLastWindowedMeasurementValue(),
                        0.0);
                assertEquals(lastMeasurement, measurement);
            }

            measurement = generateMeasurement(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp,
                    position, cnb);

            if (i == 0) {
                firstMeasurement2.setValue(measurement.getValue());
                firstMeasurement2.setUnit(measurement.getUnit());
            }

            measurements.add(new MagneticFluxDensity(measurement.getValue(), measurement.getUnit()));
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onMeasurementAdded(final WindowedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        measurementAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        windowFilled++;
    }

    @Override
    public void onReset(final WindowedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        measurementAdded = 0;
        windowFilled = 0;
        reset = 0;
    }

    private void checkLocked(final WindowedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setWindowSize(3));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(0.0));
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(LockedException.class, () -> estimator.addMeasurementAndProcess(b));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(b));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static MagneticFluxDensity generateMeasurement(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final Date timestamp, final NEDPosition position,
            final CoordinateTransformation cnb) {

        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic.getNormAsMagneticFluxDensity();
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
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
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
