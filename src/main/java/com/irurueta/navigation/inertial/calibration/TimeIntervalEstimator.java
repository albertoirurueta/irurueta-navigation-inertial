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
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;


/**
 * Estimates average time interval between processed samples.
 */
public class TimeIntervalEstimator {

    /**
     * Default total samples to be processed.
     */
    public static final int DEFAULT_TOTAL_SAMPLES = 100000;

    /**
     * Total samples to be processed to finish estimation.
     */
    private int totalSamples = DEFAULT_TOTAL_SAMPLES;

    /**
     * Listener to handle events raised by this estimator.
     */
    private TimeIntervalEstimatorListener listener;

    /**
     * Last provided timestamp expressed in seconds (s).
     */
    private Double lastTimestamp;

    /**
     * Estimated average time interval between body kinematics samples expressed in
     * seconds (s).
     */
    private double averageTimeInterval;

    /**
     * Estimated variance of time interval between body kinematics samples expressed
     * in squared seconds (s^2).
     */
    private double timeIntervalVariance;

    /**
     * Number of processed timestamp samples.
     */
    private int numberOfProcessedSamples;

    /**
     * Number of processed timestamp samples plus one.
     */
    private int numberOfProcessedSamplesPlusOne = 1;

    /**
     * Indicates that estimator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public TimeIntervalEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this estimator.
     */
    public TimeIntervalEstimator(final TimeIntervalEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public TimeIntervalEstimator(final int totalSamples) {
        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        this.totalSamples = totalSamples;
    }

    /**
     * Constructor.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @param listener     listener to handle events raised by this estimator.
     * @throws IllegalArgumentException if provided total samples is zero or negative.
     */
    public TimeIntervalEstimator(final int totalSamples, final TimeIntervalEstimatorListener listener) {
        this(totalSamples);
        this.listener = listener;
    }

    /**
     * Copy constructor.
     *
     * @param input instance to get a copy from.
     */
    public TimeIntervalEstimator(final TimeIntervalEstimator input) {
        copyFrom(input);
    }

    /**
     * Gets total samples to be processed to finish estimation.
     *
     * @return total samples to be processed to finish estimation.
     */
    public int getTotalSamples() {
        return totalSamples;
    }

    /**
     * Sets total samples to be processed to finish estimation.
     *
     * @param totalSamples total samples to be processed to finish estimation.
     * @throws LockedException if estimator is currently running.
     */
    public void setTotalSamples(final int totalSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (totalSamples <= 0) {
            throw new IllegalArgumentException();
        }

        this.totalSamples = totalSamples;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public TimeIntervalEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if this estimator is running.
     */
    public void setListener(final TimeIntervalEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets last provided timestamp expressed in seconds or null if none has been
     * provided yet.
     *
     * @return last provided timestamp or null.
     */
    public Double getLastTimestamp() {
        return lastTimestamp;
    }

    /**
     * Gets last provided timestamp or null if none has been provided yet.
     *
     * @return last provided timestamp or null.
     */
    public Time getLastTimestampAsTime() {
        return lastTimestamp != null ? new Time(lastTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Gets last provided timestamp.
     *
     * @param result instance where last provided timestamp will be stored.
     * @return true if last provided timestamp was available, false otherwise.
     */
    public boolean getLastTimestampAsTime(final Time result) {
        if (lastTimestamp != null) {
            result.setValue(lastTimestamp);
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated average time interval between body kinematics samples expressed in
     * seconds (s).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated average time interval.
     */
    public double getAverageTimeInterval() {
        return averageTimeInterval;
    }

    /**
     * Gets estimated average time interval between body kinematics samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimate average time interval.
     */
    public Time getAverageTimeIntervalAsTime() {
        return new Time(averageTimeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets estimated average time interval between body kinematics samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @param result instance where estimated average time interval will be stored.
     */
    public void getAverageTimeIntervalAsTime(final Time result) {
        result.setValue(averageTimeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets estimated variance of time interval between body kinematics samples
     * expressed in squared seconds (s^2).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated variance of time interval between body kinematics samples.
     */
    public double getTimeIntervalVariance() {
        return timeIntervalVariance;
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples expressed in seconds (s).
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated standard deviation of time interval between body kinematics
     * samples.
     */
    public double getTimeIntervalStandardDeviation() {
        return Math.sqrt(timeIntervalVariance);
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @return estimated standard deviation of time interval between body kinematics
     * samples.
     */
    public Time getTimeIntervalStandardDeviationAsTime() {
        return new Time(getTimeIntervalStandardDeviation(), TimeUnit.SECOND);
    }

    /**
     * Gets estimate standard deviation of time interval between body kinematics
     * samples.
     * Calling this method before the estimator is finished will return a provisional
     * value containing current estimation.
     *
     * @param result instance where estimated standard deviation of time interval
     *               between body kinematics samples will be stored.
     */
    public void getTimeIntervalStandardDeviationAsTime(final Time result) {
        result.setValue(getTimeIntervalStandardDeviation());
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return numberOfProcessedSamples;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Indicates whether estimator has finished the estimation.
     *
     * @return true if estimator has finished, false otherwise.
     */
    public boolean isFinished() {
        return numberOfProcessedSamples == totalSamples;
    }

    /**
     * Adds a timestamp value to current estimation.
     * If estimator is already finished, provided timestamp will be ignored.
     *
     * @param timestamp timestamp since epoch time.
     * @return true if provided timestamp has been processed, false if it has been
     * ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addTimestamp(final Time timestamp) throws LockedException {
        return addTimestamp(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Adds a timestamp value to current estimation.
     * If estimator is already finished, provided timestamp will be ignored.
     *
     * @param timestamp timestamp since epoch time expressed in seconds (s).
     * @return true if provided timestamp has been processed, false if it has been
     * ignored.
     * @throws LockedException if estimator is currently running.
     */
    public boolean addTimestamp(final double timestamp) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (isFinished()) {
            return false;
        }

        running = true;

        if (lastTimestamp == null && listener != null) {
            listener.onStart(this);
        }

        if (lastTimestamp != null) {
            final var timeInterval = timestamp - lastTimestamp;

            averageTimeInterval = averageTimeInterval * numberOfProcessedSamples / numberOfProcessedSamplesPlusOne
                    + timeInterval / numberOfProcessedSamplesPlusOne;

            final var diff = timeInterval - averageTimeInterval;
            final var diff2 = diff * diff;
            timeIntervalVariance = timeIntervalVariance * numberOfProcessedSamples / numberOfProcessedSamplesPlusOne
                    + diff2 / numberOfProcessedSamplesPlusOne;
        }

        lastTimestamp = timestamp;

        numberOfProcessedSamples++;
        numberOfProcessedSamplesPlusOne++;

        if (listener != null) {
            listener.onTimestampAdded(this);
        }

        running = false;

        if (isFinished() && listener != null) {
            listener.onFinish(this);
        }

        return true;
    }

    /**
     * Resets current estimator.
     *
     * @return true if estimator was successfully reset, false if no reset was needed.
     * @throws LockedException if estimator is currently running.
     */
    @SuppressWarnings("DuplicatedCode")
    public boolean reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (numberOfProcessedSamples == 0) {
            return false;
        }

        running = true;
        lastTimestamp = null;
        averageTimeInterval = 0.0;
        timeIntervalVariance = 0.0;
        numberOfProcessedSamples = 0;
        numberOfProcessedSamplesPlusOne = 1;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;

        return true;
    }

    /**
     * Copies instance from provided one into this one.
     *
     * @param input input instance to be copied.
     */
    public void copyFrom(final TimeIntervalEstimator input) {
        totalSamples = input.totalSamples;
        listener = input.listener;
        lastTimestamp = input.lastTimestamp;
        averageTimeInterval = input.averageTimeInterval;
        timeIntervalVariance = input.timeIntervalVariance;
        numberOfProcessedSamples = input.numberOfProcessedSamples;
        numberOfProcessedSamplesPlusOne = input.numberOfProcessedSamplesPlusOne;
        running = input.running;
    }

    /**
     * Copies current instance into provided instance.
     *
     * @param output output instance to copy to.
     */
    public void copyTo(final TimeIntervalEstimator output) {
        output.copyFrom(this);
    }
}
