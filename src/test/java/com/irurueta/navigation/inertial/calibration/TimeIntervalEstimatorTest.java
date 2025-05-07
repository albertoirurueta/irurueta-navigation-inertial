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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.LockedException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class TimeIntervalEstimatorTest implements TimeIntervalEstimatorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;
    private static final double TIME_INTERVAL_STD = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-4;

    private int start;
    private int timestampAdded;
    private int finish;
    private int reset;

    @Test
    void testConstructor() {
        // test empty constructor
        var estimator = new TimeIntervalEstimator();

        // check default values
        assertEquals(TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES, estimator.getTotalSamples());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(0.0, estimator.getAverageTimeInterval(), 0.0);
        assertEquals(0.0, estimator.getTimeIntervalVariance(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // test constructor with listener
        estimator = new TimeIntervalEstimator(this);

        // check default values
        assertEquals(TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES, estimator.getTotalSamples());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(0.0, estimator.getAverageTimeInterval(), 0.0);
        assertEquals(0.0, estimator.getTimeIntervalVariance(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // test constructor with total samples
        estimator = new TimeIntervalEstimator(1);

        // check default values
        assertEquals(1, estimator.getTotalSamples());
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(0.0, estimator.getAverageTimeInterval(), 0.0);
        assertEquals(0.0, estimator.getTimeIntervalVariance(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TimeIntervalEstimator(0));

        // test constructor with total samples and listener
        estimator = new TimeIntervalEstimator(1, this);

        // check default values
        assertEquals(1, estimator.getTotalSamples());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(0.0, estimator.getAverageTimeInterval(), 0.0);
        assertEquals(0.0, estimator.getTimeIntervalVariance(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new TimeIntervalEstimator(0, this));

        // test copy constructor
        estimator = new TimeIntervalEstimator(1, this);

        // check default values
        assertEquals(1, estimator.getTotalSamples());
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(0.0, estimator.getAverageTimeInterval(), 0.0);
        assertEquals(0.0, estimator.getTimeIntervalVariance(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        final var estimator2 = new TimeIntervalEstimator(estimator);

        // check
        assertEquals(estimator.getTotalSamples(), estimator2.getTotalSamples());
        assertSame(estimator.getListener(), estimator2.getListener());
        assertNull(estimator2.getLastTimestamp());
        assertNull(estimator2.getLastTimestampAsTime());
        assertFalse(estimator2.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), estimator2.getTimeIntervalVariance(), 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertFalse(estimator2.isRunning());
        assertFalse(estimator2.isFinished());
    }

    @Test
    void testGetSetTotalSamples() throws LockedException {
        final var estimator = new TimeIntervalEstimator();

        // check default value
        assertEquals(TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES, estimator.getTotalSamples());

        // set a new value
        estimator.setTotalSamples(1);

        // check
        assertEquals(1, estimator.getTotalSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTotalSamples(0));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new TimeIntervalEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddTimestampAndReset1() throws LockedException {
        final var estimator = new TimeIntervalEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, timestampAdded);
        assertEquals(0, finish);
        assertEquals(0, reset);
        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        final var randomizer = new GaussianRandomizer(0.0, TIME_INTERVAL_STD);
        final var totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final var lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (var i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator.getLastTimestamp());
            }

            final var noise = randomizer.nextDouble();
            final var timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator.addTimestamp(timestamp);

            assertEquals(timestamp, estimator.getLastTimestamp(), 0.0);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            assertTrue(estimator.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(timestamp, lastTimestampTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, lastTimestampTime1.getUnit());

            lastTimestamp = timestamp;
        }

        assertEquals(totalSamples, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFinished());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(totalSamples, timestampAdded);
        assertEquals(1, finish);
        assertEquals(0, reset);

        final var averageTimeInterval = estimator.getAverageTimeInterval();
        final var averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final var averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final var timeIntervalVariance = estimator.getTimeIntervalVariance();
        final var timeIntervalStandardDeviation = estimator.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final var timeIntervalStd1 = estimator.getTimeIntervalStandardDeviationAsTime();
        final var timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator.addTimestamp(0.0));

        // reset
        assertTrue(estimator.reset());

        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Test
    void testAddTimestampAndReset2() throws LockedException {
        final var estimator = new TimeIntervalEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, timestampAdded);
        assertEquals(0, finish);
        assertEquals(0, reset);
        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        final var randomizer = new GaussianRandomizer(0.0, TIME_INTERVAL_STD);
        final var totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final var lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        final var timestamp = new Time(0.0, TimeUnit.SECOND);
        for (var i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator.getLastTimestamp());
            }

            final var noise = randomizer.nextDouble();
            final var value = i * TIME_INTERVAL_SECONDS + noise;
            timestamp.setValue(value);

            assertTrue(estimator.addTimestamp(timestamp));

            assertEquals(value, estimator.getLastTimestamp(), 0.0);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            assertTrue(estimator.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(lastTimestampTime1, timestamp);

            lastTimestamp = value;
        }

        assertEquals(totalSamples, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFinished());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(totalSamples, timestampAdded);
        assertEquals(1, finish);
        assertEquals(0, reset);

        final var averageTimeInterval = estimator.getAverageTimeInterval();
        final var averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final var averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final var timeIntervalVariance = estimator.getTimeIntervalVariance();
        final var timeIntervalStandardDeviation = estimator.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final var timeIntervalStd1 = estimator.getTimeIntervalStandardDeviationAsTime();
        final var timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator.addTimestamp(timestamp));

        // reset
        assertTrue(estimator.reset());

        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());
        assertEquals(1, reset);
    }

    @Test
    void testCopyFrom() throws LockedException {
        final var estimator1 = new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        final var randomizer = new GaussianRandomizer(0.0, TIME_INTERVAL_STD);
        final var totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final var lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (var i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator1.getLastTimestamp());
            }

            final var noise = randomizer.nextDouble();
            final var timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator1.addTimestamp(timestamp);

            assertEquals(timestamp, estimator1.getLastTimestamp(), 0.0);
            assertEquals(i + 1, estimator1.getNumberOfProcessedSamples());
            assertFalse(estimator1.isRunning());

            assertTrue(estimator1.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator1.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(timestamp, lastTimestampTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, lastTimestampTime1.getUnit());

            lastTimestamp = timestamp;
        }

        assertEquals(totalSamples, estimator1.getNumberOfProcessedSamples());
        assertTrue(estimator1.isFinished());
        assertFalse(estimator1.isRunning());
        assertEquals(1, start);
        assertEquals(totalSamples, timestampAdded);
        assertEquals(1, finish);
        assertEquals(0, reset);

        final var averageTimeInterval = estimator1.getAverageTimeInterval();
        final var averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final var averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final var timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final var timeIntervalStandardDeviation = estimator1.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final var timeIntervalStd1 = estimator1.getTimeIntervalStandardDeviationAsTime();
        final var timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator1.addTimestamp(0.0));

        // copy from
        final var estimator2 = new TimeIntervalEstimator();
        estimator2.copyFrom(estimator1);

        // check
        assertEquals(estimator1.isFinished(), estimator2.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertEquals(estimator1.getTimeIntervalStandardDeviation(), estimator2.getTimeIntervalStandardDeviation(),
                0.0);
        assertEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());

        // reset
        assertTrue(estimator1.reset());

        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        // check that values are no longer equal since they have been copied
        assertNotEquals(estimator1.isFinished(), estimator2.isFinished());
        assertNotEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertNotEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertNotEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertNotEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertNotEquals(estimator1.getTimeIntervalStandardDeviation(), estimator2.getTimeIntervalStandardDeviation(),
                0.0);
        assertNotEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());
    }

    @Test
    void testCopyTo() throws LockedException {
        final var estimator1 = new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        var randomizer = new GaussianRandomizer(0.0, TIME_INTERVAL_STD);
        final var totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final var lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (var i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator1.getLastTimestamp());
            }

            final var noise = randomizer.nextDouble();
            final var timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator1.addTimestamp(timestamp);

            assertEquals(timestamp, estimator1.getLastTimestamp(), 0.0);
            assertEquals(i + 1, estimator1.getNumberOfProcessedSamples());
            assertFalse(estimator1.isRunning());

            assertTrue(estimator1.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator1.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(timestamp, lastTimestampTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, lastTimestampTime1.getUnit());

            lastTimestamp = timestamp;
        }

        assertEquals(totalSamples, estimator1.getNumberOfProcessedSamples());
        assertTrue(estimator1.isFinished());
        assertFalse(estimator1.isRunning());
        assertEquals(1, start);
        assertEquals(totalSamples, timestampAdded);
        assertEquals(1, finish);
        assertEquals(0, reset);

        final var averageTimeInterval = estimator1.getAverageTimeInterval();
        final var averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final var averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final var timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final var timeIntervalStandardDeviation = estimator1.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final var timeIntervalStd1 = estimator1.getTimeIntervalStandardDeviationAsTime();
        final var timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator1.addTimestamp(0.0));

        // copy from
        final var estimator2 = new TimeIntervalEstimator();
        estimator1.copyTo(estimator2);

        // check
        assertEquals(estimator1.isFinished(), estimator2.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertEquals(estimator1.getTimeIntervalStandardDeviation(), estimator2.getTimeIntervalStandardDeviation(),
                0.0);
        assertEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());

        // reset
        assertTrue(estimator1.reset());

        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        // check that values are no longer equal since they have been copied
        assertNotEquals(estimator1.isFinished(), estimator2.isFinished());
        assertNotEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertNotEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertNotEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertNotEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertNotEquals(estimator1.getTimeIntervalStandardDeviation(), estimator2.getTimeIntervalStandardDeviation(),
                0.0);
        assertNotEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());
    }

    @Override
    public void onStart(final TimeIntervalEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onTimestampAdded(final TimeIntervalEstimator estimator) {
        checkLocked(estimator);
        timestampAdded++;
    }

    @Override
    public void onFinish(final TimeIntervalEstimator estimator) {
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isFinished());
        finish++;
    }

    @Override
    public void onReset(final TimeIntervalEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        start = 0;
        timestampAdded = 0;
        finish = 0;
        reset = 0;
    }

    private static void checkLocked(final TimeIntervalEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTotalSamples(1));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.addTimestamp(0.0));
        assertThrows(LockedException.class, () -> assertFalse(estimator.reset()));
    }
}
