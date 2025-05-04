/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial;

import com.irurueta.units.Distance;
import com.irurueta.units.DistanceConverter;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Contains configuration parameters (usually obtained through calibration)
 * for INS/GNSS Loosely Coupled Kalman filter.
 */
public class INSTightlyCoupledKalmanConfig implements Serializable, Cloneable {

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     */
    private double gyroNoisePSD;

    /**
     * Accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3)
     */
    private double accelerometerNoisePSD;

    /**
     * Accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     */
    private double accelerometerBiasPSD;

    /**
     * Gyro bias random walk PSD (Power Spectral Density) expressed in (rad^2 * s^-3).
     */
    private double gyroBiasPSD;

    /**
     * Receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     */
    private double clockFrequencyPSD;

    /**
     * Receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     */
    private double clockPhasePSD;

    /**
     * Pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     */
    private double pseudoRangeSD;

    /**
     * Pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     */
    private double rangeRateSD;

    /**
     * Constructor.
     */
    public INSTightlyCoupledKalmanConfig() {
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public INSTightlyCoupledKalmanConfig(
            final double gyroNoisePSD, final double accelerometerNoisePSD, final double accelerometerBiasPSD,
            final double gyroBiasPSD, final double clockFrequencyPSD, final double clockPhasePSD,
            final double pseudoRangeSD, final double rangeRateSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
    }

    /**
     * Constructor.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public INSTightlyCoupledKalmanConfig(
            final double gyroNoisePSD, final double accelerometerNoisePSD, final double accelerometerBiasPSD,
            final double gyroBiasPSD, final double clockFrequencyPSD, final double clockPhasePSD,
            final Distance pseudoRangeSD, final Speed rangeRateSD) {
        setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public INSTightlyCoupledKalmanConfig(final INSTightlyCoupledKalmanConfig input) {
        copyFrom(input);
    }

    /**
     * Gets gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     *
     * @return gyro noise PSD.
     */
    public double getGyroNoisePSD() {
        return gyroNoisePSD;
    }

    /**
     * Sets gyro noise PSD (Power Spectral Density) expressed in squared radians per
     * second (rad^2/s).
     *
     * @param gyroNoisePSD gyro noise PSD.
     */
    public void setGyroNoisePSD(final double gyroNoisePSD) {
        this.gyroNoisePSD = gyroNoisePSD;
    }

    /**
     * Gets accelerometer noise PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     *
     * @return accelerometer noise PSD.
     */
    public double getAccelerometerNoisePSD() {
        return accelerometerNoisePSD;
    }

    /**
     * Sets accelerometer noise PSD (Power Spectral Density) expressed
     * in (m^2 * se^-3).
     *
     * @param accelerometerNoisePSD accelerometer noise PSD.
     */
    public void setAccelerometerNoisePSD(final double accelerometerNoisePSD) {
        this.accelerometerNoisePSD = accelerometerNoisePSD;
    }

    /**
     * Gets accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     *
     * @return accelerometer bias random walk PSD.
     */
    public double getAccelerometerBiasPSD() {
        return accelerometerBiasPSD;
    }

    /**
     * Sets accelerometer bias random walk PSD (Power Spectral Density) expressed
     * in (m^2 * s^-5).
     *
     * @param accelerometerBiasPSD accelerometer bias random walk PSD.
     */
    public void setAccelerometerBiasPSD(final double accelerometerBiasPSD) {
        this.accelerometerBiasPSD = accelerometerBiasPSD;
    }

    /**
     * Gets gyro bias random walk PSD (Power Spectral Density) expressed in
     * (rad^2 * s^-3).
     *
     * @return gyro bias random walk PSD.
     */
    public double getGyroBiasPSD() {
        return gyroBiasPSD;
    }

    /**
     * Sets gyro bias random walk PSD (Power Spectral Density) expressed in
     * (rad^2 * s^-3).
     *
     * @param gyroBiasPSD gyro bias random walk PSD.
     */
    public void setGyroBiasPSD(final double gyroBiasPSD) {
        this.gyroBiasPSD = gyroBiasPSD;
    }

    /**
     * Gets receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     *
     * @return receiver clock frequency-drift PSD.
     */
    public double getClockFrequencyPSD() {
        return clockFrequencyPSD;
    }

    /**
     * Sets receiver clock frequency-drift PSD (Power Spectral Density) expressed
     * in (m^2/s^3).
     *
     * @param clockFrequencyPSD clock frequency-drift PSD.
     */
    public void setClockFrequencyPSD(final double clockFrequencyPSD) {
        this.clockFrequencyPSD = clockFrequencyPSD;
    }

    /**
     * Gets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     *
     * @return receiver clock phase-drift PSD.
     */
    public double getClockPhasePSD() {
        return clockPhasePSD;
    }

    /**
     * Sets receiver clock phase-drift PSD (Power Spectral Density) expressed in
     * squared meters per second (m^2/s).
     *
     * @param clockPhasePSD receiver clock phase-drift PSD.
     */
    public void setClockPhasePSD(final double clockPhasePSD) {
        this.clockPhasePSD = clockPhasePSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     *
     * @return pseudo-range measurement noise SD.
     */
    public double getPseudoRangeSD() {
        return pseudoRangeSD;
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation) expressed in
     * meters (m).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final double pseudoRangeSD) {
        this.pseudoRangeSD = pseudoRangeSD;
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public double getRangeRateSD() {
        return rangeRateSD;
    }

    /**
     * Sets pseudo-range rate measurement noise SD (Standard Deviation) expressed
     * in meters per second (m/s).
     *
     * @param rangeRateSD pseudo-range rate measurement noise SD.
     */
    public void setRangeRateSD(final double rangeRateSD) {
        this.rangeRateSD = rangeRateSD;
    }

    /**
     * Sets configuration parameters.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public void setValues(
            final double gyroNoisePSD, final double accelerometerNoisePSD, final double accelerometerBiasPSD,
            final double gyroBiasPSD, final double clockFrequencyPSD, final double clockPhasePSD,
            final double pseudoRangeSD, final double rangeRateSD) {
        this.gyroNoisePSD = gyroNoisePSD;
        this.accelerometerNoisePSD = accelerometerNoisePSD;
        this.accelerometerBiasPSD = accelerometerBiasPSD;
        this.gyroBiasPSD = gyroBiasPSD;
        this.clockFrequencyPSD = clockFrequencyPSD;
        this.clockPhasePSD = clockPhasePSD;
        this.pseudoRangeSD = pseudoRangeSD;
        this.rangeRateSD = rangeRateSD;
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range measurement noise SD will be stored.
     */
    public void getPseudoRangeSDDistance(final Distance result) {
        result.setValue(pseudoRangeSD);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range measurement noise SD.
     */
    public Distance getPseudoRangeSDDistance() {
        return new Distance(pseudoRangeSD, DistanceUnit.METER);
    }

    /**
     * Sets pseudo-range measurement noise SD (Standard Deviation).
     *
     * @param pseudoRangeSD pseudo-range measurement noise SD.
     */
    public void setPseudoRangeSD(final Distance pseudoRangeSD) {
        this.pseudoRangeSD = DistanceConverter.convert(pseudoRangeSD.getValue().doubleValue(), pseudoRangeSD.getUnit(),
                DistanceUnit.METER);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param result instance where pseudo-range rate measurement noise SD will be
     *               stored.
     */
    public void getRangeRateSDSpeed(final Speed result) {
        result.setValue(rangeRateSD);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @return pseudo-range rate measurement noise SD.
     */
    public Speed getRangeRateSDSpeed() {
        return new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets pseudo-range rate measurement noise SD (Standard Deviation).
     *
     * @param rangeRateSD pseudo-range rate measurement noise SD.
     */
    public void setRangeRateSD(final Speed rangeRateSD) {
        this.rangeRateSD = SpeedConverter.convert(rangeRateSD.getValue().doubleValue(), rangeRateSD.getUnit(),
                SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets configuration parameters.
     *
     * @param gyroNoisePSD          gyro noise PSD (Power Spectral Density) expressed
     *                              in squared radians per second (rad^2/s).
     * @param accelerometerNoisePSD accelerometer noise PSD (Power Spectral Density)
     *                              expressed in (m^2 * s^-3).
     * @param accelerometerBiasPSD  accelerometer bias random walk PSD (Power Spectral
     *                              Density) expressed in (m^2 * s^-5).
     * @param gyroBiasPSD           gyro bias random walk PSD (Power Spectral Density)
     *                              expressed in (rad^2 * s^-3).
     * @param clockFrequencyPSD     receiver clock frequency-drift PSD (Power Spectral
     *                              Density) expressed in (m^2/s^3).
     * @param clockPhasePSD         receiver clock phase-drift PSD (Power Spectral
     *                              Density) expressed in squared meters per second
     *                              (m^2/s).
     * @param pseudoRangeSD         pseudo-range measurement noise SD (Standard
     *                              Deviation) expressed in meters (m).
     * @param rangeRateSD           pseudo-range rate measurement noise SD (Standard
     *                              Deviation) expressed in meters per second (m/s).
     */
    public void setValues(
            final double gyroNoisePSD, final double accelerometerNoisePSD, final double accelerometerBiasPSD,
            final double gyroBiasPSD, final double clockFrequencyPSD, final double clockPhasePSD,
            final Distance pseudoRangeSD, final Speed rangeRateSD) {
        this.gyroNoisePSD = gyroNoisePSD;
        this.accelerometerNoisePSD = accelerometerNoisePSD;
        this.accelerometerBiasPSD = accelerometerBiasPSD;
        this.gyroBiasPSD = gyroBiasPSD;
        this.clockFrequencyPSD = clockFrequencyPSD;
        this.clockPhasePSD = clockPhasePSD;
        setPseudoRangeSD(pseudoRangeSD);
        setRangeRateSD(rangeRateSD);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final INSTightlyCoupledKalmanConfig output) {
        output.gyroNoisePSD = gyroNoisePSD;
        output.accelerometerNoisePSD = accelerometerNoisePSD;
        output.accelerometerBiasPSD = accelerometerBiasPSD;
        output.gyroBiasPSD = gyroBiasPSD;
        output.clockFrequencyPSD = clockFrequencyPSD;
        output.clockPhasePSD = clockPhasePSD;
        output.pseudoRangeSD = pseudoRangeSD;
        output.rangeRateSD = rangeRateSD;
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final INSTightlyCoupledKalmanConfig input) {
        gyroNoisePSD = input.gyroNoisePSD;
        accelerometerNoisePSD = input.accelerometerNoisePSD;
        accelerometerBiasPSD = input.accelerometerBiasPSD;
        gyroBiasPSD = input.gyroBiasPSD;
        clockFrequencyPSD = input.clockFrequencyPSD;
        clockPhasePSD = input.clockPhasePSD;
        pseudoRangeSD = input.pseudoRangeSD;
        rangeRateSD = input.rangeRateSD;
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
    }

    /**
     * Checks if provided object is a INSTightlyCoupledKalmanConfig having exactly
     * the same contents as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final var other = (INSTightlyCoupledKalmanConfig) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final INSTightlyCoupledKalmanConfig other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final INSTightlyCoupledKalmanConfig other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(gyroNoisePSD - other.gyroNoisePSD) <= threshold
                && Math.abs(accelerometerNoisePSD - other.accelerometerNoisePSD) <= threshold
                && Math.abs(accelerometerBiasPSD - other.accelerometerBiasPSD) <= threshold
                && Math.abs(gyroBiasPSD - other.gyroBiasPSD) <= threshold
                && Math.abs(clockFrequencyPSD - other.clockFrequencyPSD) <= threshold
                && Math.abs(clockPhasePSD - other.clockPhasePSD) <= threshold
                && Math.abs(pseudoRangeSD - other.pseudoRangeSD) <= threshold
                && Math.abs(rangeRateSD - other.rangeRateSD) <= threshold;
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (INSTightlyCoupledKalmanConfig) super.clone();
        copyTo(result);
        return result;
    }
}
