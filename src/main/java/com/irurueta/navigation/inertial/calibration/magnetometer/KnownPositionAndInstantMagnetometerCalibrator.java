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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;

import java.io.IOException;
import java.util.Collection;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;

/**
 * Estimates magnetometer hard-iron biases, cross couplings and scaling
 * factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared
 * error solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single known
 * position and instant must be taken at 10 different unknown orientations
 * when common z-axis is assumed, otherwise at least 13
 * measurements are required.
 * <p>
 * Measured magnetic flux density is assumed to follow the model shown below:
 * <pre>
 *     mBmeas = bm + (I + Mm) * mBtrue + w
 * </pre>
 * Where:
 * - mBmeas is the measured magnetic flux density. This is a 3x1 vector.
 * - bm is magnetometer hard-iron bias. Ideally, on a perfect magnetometer,
 * this should be a 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mm is the 3x3 soft-iron matrix containing cross-couplings and scaling
 * factors. Ideally, on a perfect magnetometer, this should be a 3x3 zero
 * matrix.
 * - mBtrue is ground-truth magnetic flux density. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 * Notice that this calibrator assumes that all measurements are taken in
 * a short span of time, where Earth magnetic field can be assumed to be
 * constant at provided location and instant.
 */
public class KnownPositionAndInstantMagnetometerCalibrator extends
        BaseMagneticFluxDensityNormMagnetometerCalibrator<KnownPositionAndInstantMagnetometerCalibrator,
                KnownPositionAndInstantMagnetometerCalibratorListener> {

    /**
     * Position where body magnetic flux density measurements have been
     * taken.
     */
    private NEDPosition mPosition;

    /**
     * Timestamp expressed as decimal year, where magnetic flux density
     * measurements have been measured.
     */
    private Double mYear = convertTime(System.currentTimeMillis());

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * Constructor.
     */
    public KnownPositionAndInstantMagnetometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final WorldMagneticModel magneticModel) {
        super();
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final double[] initialHardIron) {
        super(initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final Matrix initialHardIron) {
        super(initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final Matrix initialHardIron, final Matrix initialMm) {
        super(initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final NEDPosition position) {
        super();
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        super(measurements, initialHardIron);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        super(measurements, initialHardIron);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        super(measurements, initialHardIron, initialMm);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, initialMm, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(final ECEFPosition position) {
        this(convertPosition(position));
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(convertPosition(position), measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(convertPosition(position), measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(convertPosition(position), measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(convertPosition(position), measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final KnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken.
     *
     * @return position where body magnetic flux density measurements have
     * been taken.
     */
    public NEDPosition getNedPosition() {
        return mPosition;
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final NEDPosition position) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        mPosition = position;
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @return position where body magnetic flux density measurements have
     * been taken or null if not available.
     */
    public ECEFPosition getEcefPosition() {
        if (mPosition != null) {
            final ECEFPosition result = new ECEFPosition();
            getEcefPosition(result);
            return result;
        } else {
            return null;
        }
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if ECEF position could be computed, false otherwise.
     */
    public boolean getEcefPosition(final ECEFPosition result) {
        if (mPosition != null) {
            final ECEFVelocity velocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    mPosition.getLatitude(), mPosition.getLongitude(), mPosition.getHeight(),
                    0.0, 0.0, 0.0, result, velocity);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param position position where body magnetic flux density have been
     *                 taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final ECEFPosition position) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        mPosition = convertPosition(position);
    }

    /**
     * Gets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @return timestamp expressed as decimal year or null if not defined.
     */
    public Double getYear() {
        return mYear;
    }

    /**
     * Sets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @param year timestamp expressed as decimal year.
     * @throws LockedException if calibrator is currently running.
     */
    public void setYear(final Double year) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        mYear = year;
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param timestampMillis a timestamp expressed in milliseconds since
     *                        epoch time (January 1st, 1970 at midnight).
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Long timestampMillis) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        mYear = convertTime(timestampMillis);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param date a date instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Date date) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        mYear = convertTime(date);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param calendar a calendar instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final GregorianCalendar calendar) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        mYear = convertTime(calendar);
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mPosition != null && mYear != null;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    public WorldMagneticModel getMagneticModel() {
        return mMagneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMagneticModel(final WorldMagneticModel magneticModel) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
    }

    /**
     * Called before calibration occurs.
     * This can be overridden by subclasses.
     *
     * @throws CalibrationException if anything fails.
     */
    @Override
    protected void onBeforeCalibrate() throws CalibrationException {
        computeGroundTruthMagneticFluxDensityNorm();
    }

    /**
     * Computes expected ground truth magnetic flux density norm based on World Magnetic Model.
     * Notice that actual magnetic field measured by devices might differ from this value, since it might be attenuated
     * or other devices might interfere.
     *
     * @throws CalibrationException if world magnetic model cannot be loaded.
     */
    private void computeGroundTruthMagneticFluxDensityNorm() throws CalibrationException {
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            try {
                wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            } catch (final IOException e) {
                throw new CalibrationException(e);
            }
        }

        final NEDPosition position = getNedPosition();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(position, mYear);
        mGroundTruthMagneticFluxDensityNorm = earthB.getNorm();
    }

    /**
     * Converts a time instance expressed in milliseconds since epoch time
     * (January 1st, 1970 at midnight) to a decimal year.
     *
     * @param timestampMillis milliseconds value to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Long timestampMillis) {
        if (timestampMillis == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestampMillis);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained ina date object to a
     * decimal year.
     *
     * @param date a time instance to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Date date) {
        if (date == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(date);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained in a gregorian calendar to a
     * decimal year.
     *
     * @param calendar calendar containing a specific instant to be
     *                 converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final GregorianCalendar calendar) {
        if (calendar == null) {
            return null;
        }

        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }

    /**
     * Converts provided ECEF position to position expressed in NED
     * coordinates.
     *
     * @param position ECEF position to be converted.
     * @return converted position expressed in NED coordinates.
     */
    private static NEDPosition convertPosition(final ECEFPosition position) {
        final NEDVelocity velocity = new NEDVelocity();
        final NEDPosition result = new NEDPosition();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                position.getX(), position.getY(), position.getZ(),
                0.0, 0.0, 0.0, result, velocity);
        return result;
    }
}
