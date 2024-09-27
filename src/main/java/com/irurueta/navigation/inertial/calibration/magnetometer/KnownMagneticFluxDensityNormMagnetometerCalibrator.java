/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.util.Collection;

/**
 * Estimates magnetometer hard-iron biases, cross couplings and scaling factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared
 * error solution.
 * <p>
 * To use this calibrator at least 10 measurements with known magnetic field norm at
 * an unknown position and instant must be taken at 10 different unknown orientations
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
public class KnownMagneticFluxDensityNormMagnetometerCalibrator extends
        BaseMagneticFluxDensityNormMagnetometerCalibrator<KnownMagneticFluxDensityNormMagnetometerCalibrator,
                KnownMagneticFluxDensityNormMagnetometerCalibratorListener> {

    /**
     * Constructor.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(final double[] initialHardIron) {
        super(initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(final Matrix initialHardIron) {
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(final Matrix initialHardIron, final Matrix initialMm) {
        super(initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
        super(measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
        super(measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm) {
        super(measurements, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
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
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(final Double groundTruthMagneticFluxDensityNorm) {
        super(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(groundTruthMagneticFluxDensityNorm, measurements);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed) {
        super(groundTruthMagneticFluxDensityNorm, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix initialHardIron, final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public KnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made, expressed in Teslas (T).
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthMagneticFluxDensityNorm(final Double groundTruthMagneticFluxDensityNorm)
            throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthMagneticFluxDensityNorm(final MagneticFluxDensity groundTruthMagneticFluxDensityNorm)
            throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        if (groundTruthMagneticFluxDensityNorm != null) {
            internalSetGroundTruthMagneticFluxDensityNorm(MagneticFluxDensityConverter.convert(
                    groundTruthMagneticFluxDensityNorm.getValue().doubleValue(),
                    groundTruthMagneticFluxDensityNorm.getUnit(),
                    MagneticFluxDensityUnit.TESLA));
        } else {
            internalSetGroundTruthMagneticFluxDensityNorm(null);
        }
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && getGroundTruthMagneticFluxDensityNorm() != null;
    }
}
