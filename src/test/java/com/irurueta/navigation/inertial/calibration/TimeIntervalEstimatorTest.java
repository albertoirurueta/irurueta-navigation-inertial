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
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class TimeIntervalEstimatorTest implements TimeIntervalEstimatorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;
    private static final double TIME_INTERVAL_STD = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-4;

    private int mStart;
    private int mTimestampAdded;
    private int mFinish;
    private int mReset;

    @Test
    public void testConstructor() {
        // test empty constructor
        TimeIntervalEstimator estimator = new TimeIntervalEstimator();

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

        final TimeIntervalEstimator estimator2 = new TimeIntervalEstimator(estimator);

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
    public void testGetSetTotalSamples() throws LockedException {
        final TimeIntervalEstimator estimator = new TimeIntervalEstimator();

        // check default value
        assertEquals(TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES, estimator.getTotalSamples());

        // set new value
        estimator.setTotalSamples(1);

        // check
        assertEquals(1, estimator.getTotalSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTotalSamples(0));
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final TimeIntervalEstimator estimator = new TimeIntervalEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTimestampAndReset1() throws LockedException {
        final TimeIntervalEstimator estimator = new TimeIntervalEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mTimestampAdded);
        assertEquals(0, mFinish);
        assertEquals(0, mReset);
        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        final GaussianRandomizer randomizer = new GaussianRandomizer(new Random(), 0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator.getLastTimestamp());
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

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
        assertEquals(1, mStart);
        assertEquals(totalSamples, mTimestampAdded);
        assertEquals(1, mFinish);
        assertEquals(0, mReset);

        final double averageTimeInterval = estimator.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator.getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
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
        assertEquals(1, mReset);
    }

    @Test
    public void testAddTimestampAndReset2() throws LockedException {
        final TimeIntervalEstimator estimator = new TimeIntervalEstimator(this);

        reset();
        assertEquals(0, mStart);
        assertEquals(0, mTimestampAdded);
        assertEquals(0, mFinish);
        assertEquals(0, mReset);
        assertFalse(estimator.isFinished());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        final GaussianRandomizer randomizer = new GaussianRandomizer(new Random(), 0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        final Time timestamp = new Time(0.0, TimeUnit.SECOND);
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator.getLastTimestamp());
            }

            final double noise = randomizer.nextDouble();
            final double value = i * TIME_INTERVAL_SECONDS + noise;
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
        assertEquals(1, mStart);
        assertEquals(totalSamples, mTimestampAdded);
        assertEquals(1, mFinish);
        assertEquals(0, mReset);

        final double averageTimeInterval = estimator.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator.getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
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
        assertEquals(1, mReset);
    }

    @Test
    public void testCopyFrom() throws LockedException {
        final TimeIntervalEstimator estimator1 = new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        GaussianRandomizer randomizer = new GaussianRandomizer(new Random(), 0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator1.getLastTimestamp());
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

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
        assertEquals(1, mStart);
        assertEquals(totalSamples, mTimestampAdded);
        assertEquals(1, mFinish);
        assertEquals(0, mReset);

        final double averageTimeInterval = estimator1.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator1.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator1.getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator1.addTimestamp(0.0));

        // copy from
        final TimeIntervalEstimator estimator2 = new TimeIntervalEstimator();
        estimator2.copyFrom(estimator1);

        // check
        assertEquals(estimator1.isFinished(), estimator2.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertEquals(estimator1.getTimeIntervalStandardDeviation(),
                estimator2.getTimeIntervalStandardDeviation(), 0.0);
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
    public void testCopyTo() throws LockedException {
        final TimeIntervalEstimator estimator1 = new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(0, estimator1.getNumberOfProcessedSamples());
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        GaussianRandomizer randomizer = new GaussianRandomizer(new Random(), 0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(lastTimestamp, estimator1.getLastTimestamp());
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

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
        assertEquals(1, mStart);
        assertEquals(totalSamples, mTimestampAdded);
        assertEquals(1, mFinish);
        assertEquals(0, mReset);

        final double averageTimeInterval = estimator1.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval, averageTimeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, averageTimeInterval1.getUnit());

        assertEquals(TIME_INTERVAL_SECONDS, averageTimeInterval, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator1.getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalStandardDeviation * timeIntervalStandardDeviation, timeIntervalVariance,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator1.getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStandardDeviation, timeIntervalStd1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeIntervalStd1.getUnit());

        assertEquals(TIME_INTERVAL_STD, timeIntervalStandardDeviation, LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator1.addTimestamp(0.0));

        // copy from
        final TimeIntervalEstimator estimator2 = new TimeIntervalEstimator();
        estimator1.copyTo(estimator2);

        // check
        assertEquals(estimator1.isFinished(), estimator2.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertEquals(estimator1.getTimeIntervalStandardDeviation(), estimator2.getTimeIntervalStandardDeviation(), 0.0);
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
        mStart++;
    }

    @Override
    public void onTimestampAdded(final TimeIntervalEstimator estimator) {
        checkLocked(estimator);
        mTimestampAdded++;
    }

    @Override
    public void onFinish(final TimeIntervalEstimator estimator) {
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isFinished());
        mFinish++;
    }

    @Override
    public void onReset(final TimeIntervalEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mTimestampAdded = 0;
        mFinish = 0;
        mReset = 0;
    }

    private static void checkLocked(final TimeIntervalEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTotalSamples(1));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.addTimestamp(0.0));
        assertThrows(LockedException.class, () -> assertFalse(estimator.reset()));
    }
}
