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
        assertEquals(estimator.getTotalSamples(),
                TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), 0.0, 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // test constructor with listener
        estimator = new TimeIntervalEstimator(this);

        // check default values
        assertEquals(estimator.getTotalSamples(),
                TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), 0.0, 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // test constructor with total samples
        estimator = new TimeIntervalEstimator(1);

        // check default values
        assertEquals(estimator.getTotalSamples(), 1);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), 0.0, 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new TimeIntervalEstimator(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test constructor with total samples and listener
        estimator = new TimeIntervalEstimator(1, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), 1);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), 0.0, 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new TimeIntervalEstimator(0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);

        // test copy constructor
        estimator = new TimeIntervalEstimator(1, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), 1);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastTimestamp());
        assertNull(estimator.getLastTimestampAsTime());
        assertFalse(estimator.getLastTimestampAsTime(null));
        assertEquals(estimator.getAverageTimeInterval(), 0.0, 0.0);
        assertEquals(estimator.getTimeIntervalVariance(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
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
        assertEquals(estimator.getTotalSamples(),
                TimeIntervalEstimator.DEFAULT_TOTAL_SAMPLES);

        // set new value
        estimator.setTotalSamples(1);

        // check
        assertEquals(estimator.getTotalSamples(), 1);

        // Force IllegalArgumentException
        try {
            estimator.setTotalSamples(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final TimeIntervalEstimator estimator = new TimeIntervalEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testAddTimestampAndReset1() throws LockedException {
        final TimeIntervalEstimator estimator =
                new TimeIntervalEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTimestampAdded, 0);
        assertEquals(mFinish, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        GaussianRandomizer randomizer = new GaussianRandomizer(new Random(),
                0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(estimator.getLastTimestamp(), lastTimestamp);
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator.addTimestamp(timestamp);

            assertEquals(estimator.getLastTimestamp(), timestamp, 0.0);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            assertTrue(estimator.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(lastTimestampTime1.getValue().doubleValue(), timestamp,
                    0.0);
            assertEquals(lastTimestampTime1.getUnit(), TimeUnit.SECOND);

            lastTimestamp = timestamp;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), totalSamples);
        assertTrue(estimator.isFinished());
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTimestampAdded, totalSamples);
        assertEquals(mFinish, 1);
        assertEquals(mReset, 0);

        final double averageTimeInterval = estimator.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval1.getValue().doubleValue(),
                averageTimeInterval, 0.0);
        assertEquals(averageTimeInterval1.getUnit(), TimeUnit.SECOND);

        assertEquals(averageTimeInterval, TIME_INTERVAL_SECONDS, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator
                .getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalVariance,
                timeIntervalStandardDeviation * timeIntervalStandardDeviation,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator
                .getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStd1.getValue().doubleValue(),
                timeIntervalStandardDeviation, 0.0);
        assertEquals(timeIntervalStd1.getUnit(), TimeUnit.SECOND);

        assertEquals(timeIntervalStandardDeviation, TIME_INTERVAL_STD,
                LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator.addTimestamp(0.0));

        // reset
        assertTrue(estimator.reset());

        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Test
    public void testAddTimestampAndReset2() throws LockedException {
        final TimeIntervalEstimator estimator =
                new TimeIntervalEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTimestampAdded, 0);
        assertEquals(mFinish, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());

        final GaussianRandomizer randomizer = new GaussianRandomizer(new Random(),
                0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        final Time timestamp = new Time(0.0, TimeUnit.SECOND);
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(estimator.getLastTimestamp(), lastTimestamp);
            }

            final double noise = randomizer.nextDouble();
            final double value = i * TIME_INTERVAL_SECONDS + noise;
            timestamp.setValue(value);

            assertTrue(estimator.addTimestamp(timestamp));

            assertEquals(estimator.getLastTimestamp(), value, 0.0);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            assertTrue(estimator.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(lastTimestampTime1, timestamp);

            lastTimestamp = value;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), totalSamples);
        assertTrue(estimator.isFinished());
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTimestampAdded, totalSamples);
        assertEquals(mFinish, 1);
        assertEquals(mReset, 0);

        final double averageTimeInterval = estimator.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval1.getValue().doubleValue(),
                averageTimeInterval, 0.0);
        assertEquals(averageTimeInterval1.getUnit(), TimeUnit.SECOND);

        assertEquals(averageTimeInterval, TIME_INTERVAL_SECONDS, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator
                .getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalVariance,
                timeIntervalStandardDeviation * timeIntervalStandardDeviation,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator
                .getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStd1.getValue().doubleValue(),
                timeIntervalStandardDeviation, 0.0);
        assertEquals(timeIntervalStd1.getUnit(), TimeUnit.SECOND);

        assertEquals(timeIntervalStandardDeviation, TIME_INTERVAL_STD,
                LARGE_ABSOLUTE_ERROR);

        assertFalse(estimator.addTimestamp(timestamp));

        // reset
        assertTrue(estimator.reset());

        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTimestamp());
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Test
    public void testCopyFrom() throws LockedException {
        final TimeIntervalEstimator estimator1 =
                new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), 0);
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        GaussianRandomizer randomizer = new GaussianRandomizer(new Random(),
                0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(estimator1.getLastTimestamp(), lastTimestamp);
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator1.addTimestamp(timestamp);

            assertEquals(estimator1.getLastTimestamp(), timestamp, 0.0);
            assertEquals(estimator1.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator1.isRunning());

            assertTrue(estimator1.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator1.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(lastTimestampTime1.getValue().doubleValue(), timestamp,
                    0.0);
            assertEquals(lastTimestampTime1.getUnit(), TimeUnit.SECOND);

            lastTimestamp = timestamp;
        }

        assertEquals(estimator1.getNumberOfProcessedSamples(), totalSamples);
        assertTrue(estimator1.isFinished());
        assertFalse(estimator1.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTimestampAdded, totalSamples);
        assertEquals(mFinish, 1);
        assertEquals(mReset, 0);

        final double averageTimeInterval = estimator1.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval1.getValue().doubleValue(),
                averageTimeInterval, 0.0);
        assertEquals(averageTimeInterval1.getUnit(), TimeUnit.SECOND);

        assertEquals(averageTimeInterval, TIME_INTERVAL_SECONDS, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator1
                .getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalVariance,
                timeIntervalStandardDeviation * timeIntervalStandardDeviation,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator1
                .getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStd1.getValue().doubleValue(),
                timeIntervalStandardDeviation, 0.0);
        assertEquals(timeIntervalStd1.getUnit(), TimeUnit.SECOND);

        assertEquals(timeIntervalStandardDeviation, TIME_INTERVAL_STD,
                LARGE_ABSOLUTE_ERROR);

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
        assertEquals(estimator1.getNumberOfProcessedSamples(), 0);
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        // check that values are no longer equal since they have been copied
        assertNotEquals(estimator1.isFinished(), estimator2.isFinished());
        assertNotEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertNotEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertNotEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertNotEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertNotEquals(estimator1.getTimeIntervalStandardDeviation(),
                estimator2.getTimeIntervalStandardDeviation(), 0.0);
        assertNotEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());
    }

    @Test
    public void testCopyTo() throws LockedException {
        final TimeIntervalEstimator estimator1 =
                new TimeIntervalEstimator(this);

        reset();
        assertFalse(estimator1.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), 0);
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        GaussianRandomizer randomizer = new GaussianRandomizer(new Random(),
                0.0, TIME_INTERVAL_STD);
        final int totalSamples = estimator1.getTotalSamples();
        Double lastTimestamp = null;
        final Time lastTimestampTime1 = new Time(0.0, TimeUnit.MINUTE);
        Time lastTimestampTime2;
        for (int i = 0; i < totalSamples; i++) {
            if (lastTimestamp != null) {
                assertEquals(estimator1.getLastTimestamp(), lastTimestamp);
            }

            final double noise = randomizer.nextDouble();
            final double timestamp = i * TIME_INTERVAL_SECONDS + noise;

            estimator1.addTimestamp(timestamp);

            assertEquals(estimator1.getLastTimestamp(), timestamp, 0.0);
            assertEquals(estimator1.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator1.isRunning());

            assertTrue(estimator1.getLastTimestampAsTime(lastTimestampTime1));
            lastTimestampTime2 = estimator1.getLastTimestampAsTime();

            assertEquals(lastTimestampTime1, lastTimestampTime2);
            assertEquals(lastTimestampTime1.getValue().doubleValue(), timestamp,
                    0.0);
            assertEquals(lastTimestampTime1.getUnit(), TimeUnit.SECOND);

            lastTimestamp = timestamp;
        }

        assertEquals(estimator1.getNumberOfProcessedSamples(), totalSamples);
        assertTrue(estimator1.isFinished());
        assertFalse(estimator1.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTimestampAdded, totalSamples);
        assertEquals(mFinish, 1);
        assertEquals(mReset, 0);

        final double averageTimeInterval = estimator1.getAverageTimeInterval();
        final Time averageTimeInterval1 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getAverageTimeIntervalAsTime(averageTimeInterval1);
        final Time averageTimeInterval2 = estimator1.getAverageTimeIntervalAsTime();

        assertEquals(averageTimeInterval1, averageTimeInterval2);
        assertEquals(averageTimeInterval1.getValue().doubleValue(),
                averageTimeInterval, 0.0);
        assertEquals(averageTimeInterval1.getUnit(), TimeUnit.SECOND);

        assertEquals(averageTimeInterval, TIME_INTERVAL_SECONDS, ABSOLUTE_ERROR);

        final double timeIntervalVariance = estimator1.getTimeIntervalVariance();
        final double timeIntervalStandardDeviation = estimator1
                .getTimeIntervalStandardDeviation();

        assertEquals(timeIntervalVariance,
                timeIntervalStandardDeviation * timeIntervalStandardDeviation,
                SMALL_ABSOLUTE_ERROR);

        final Time timeIntervalStd1 = estimator1
                .getTimeIntervalStandardDeviationAsTime();
        final Time timeIntervalStd2 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator1.getTimeIntervalStandardDeviationAsTime(timeIntervalStd2);

        assertEquals(timeIntervalStd1, timeIntervalStd2);
        assertEquals(timeIntervalStd1.getValue().doubleValue(),
                timeIntervalStandardDeviation, 0.0);
        assertEquals(timeIntervalStd1.getUnit(), TimeUnit.SECOND);

        assertEquals(timeIntervalStandardDeviation, TIME_INTERVAL_STD,
                LARGE_ABSOLUTE_ERROR);

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
        assertEquals(estimator1.getTimeIntervalStandardDeviation(),
                estimator2.getTimeIntervalStandardDeviation(), 0.0);
        assertEquals(estimator1.getTimeIntervalStandardDeviationAsTime(),
                estimator2.getTimeIntervalStandardDeviationAsTime());

        // reset
        assertTrue(estimator1.reset());

        assertFalse(estimator1.isFinished());
        assertEquals(estimator1.getNumberOfProcessedSamples(), 0);
        assertNull(estimator1.getLastTimestamp());
        assertFalse(estimator1.isRunning());

        // check that values are no longer equal since they have been copied
        assertNotEquals(estimator1.isFinished(), estimator2.isFinished());
        assertNotEquals(estimator1.getNumberOfProcessedSamples(), estimator2.getNumberOfProcessedSamples());
        assertNotEquals(estimator1.getLastTimestamp(), estimator2.getLastTimestamp());
        assertEquals(estimator2.isRunning(), estimator1.isRunning());
        assertNotEquals(estimator1.getAverageTimeInterval(), estimator2.getAverageTimeInterval(), 0.0);
        assertNotEquals(estimator1.getAverageTimeIntervalAsTime(), estimator2.getAverageTimeIntervalAsTime());
        assertNotEquals(estimator1.getTimeIntervalStandardDeviation(),
                estimator2.getTimeIntervalStandardDeviation(), 0.0);
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

    private void checkLocked(final TimeIntervalEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setTotalSamples(1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTimestamp(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            assertFalse(estimator.reset());
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }
}
