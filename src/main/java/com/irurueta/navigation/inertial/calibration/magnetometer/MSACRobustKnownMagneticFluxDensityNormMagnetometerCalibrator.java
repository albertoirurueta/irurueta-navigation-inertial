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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates magnetometer hard-iron biases, cross couplings and
 * scaling factors using MSAC algorithm.
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
public class MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator extends
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator {
    /**
     * Constant defining default threshold to determine whether samples are
     * inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold to determine whether samples are inliers or not when
     * testing possible estimation solutions.
     */
    private double mThreshold = DEFAULT_THRESHOLD;

    /**
     * Constructor.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] initialHardIron) {
        super(initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Matrix initialHardIron, final Matrix initialMm) {
        super(initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm) {
        super(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final boolean commonAxisUsed) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final double[] initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Matrix initialHardIron, final Matrix initialMm) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm) {
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
    public MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Returns threshold to determine whether samples are inliers or not.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not.
     *
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than
     *                                  zero.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }

    /**
     * Estimates magnetometer calibration parameters containing hard-iron
     * bias and soft-iron scale factors and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (mRunning) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final MSACRobustEstimator<PreliminaryResult> innerEstimator =
                new MSACRobustEstimator<>(
                        new MSACRobustEstimatorListener<PreliminaryResult>() {
                            @Override
                            public double getThreshold() {
                                return mThreshold;
                            }

                            @Override
                            public int getTotalSamples() {
                                return mMeasurements.size();
                            }

                            @Override
                            public int getSubsetSize() {
                                return mPreliminarySubsetSize;
                            }

                            @Override
                            public void estimatePreliminarSolutions(
                                    final int[] samplesIndices,
                                    final List<PreliminaryResult> solutions) {
                                computePreliminarySolutions(samplesIndices, solutions);
                            }

                            @Override
                            public double computeResidual(
                                    final PreliminaryResult currentEstimation,
                                    final int i) {
                                return computeError(mMeasurements.get(i), currentEstimation);
                            }

                            @Override
                            public boolean isReady() {
                                return MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.super.isReady();
                            }

                            @Override
                            public void onEstimateStart(final RobustEstimator<PreliminaryResult> estimator) {
                                // no action needed
                            }

                            @Override
                            public void onEstimateEnd(final RobustEstimator<PreliminaryResult> estimator) {
                                // no action needed
                            }

                            @Override
                            public void onEstimateNextIteration(final RobustEstimator<PreliminaryResult> estimator,
                                    final int iteration) {
                                if (mListener != null) {
                                    mListener.onCalibrateNextIteration(
                                            MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.this,
                                            iteration);
                                }
                            }

                            @Override
                            public void onEstimateProgressChange(final RobustEstimator<PreliminaryResult> estimator,
                                    final float progress) {
                                if (mListener != null) {
                                    mListener.onCalibrateProgressChange(
                                            MSACRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.this,
                                            progress);
                                }
                            }
                        });

        try {
            mRunning = true;

            if (mListener != null) {
                mListener.onCalibrateStart(this);
            }

            mInliersData = null;

            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            final PreliminaryResult preliminaryResult = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

            if (mListener != null) {
                mListener.onCalibrateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (final RobustEstimatorException e) {
            throw new CalibrationException(e);
        } finally {
            mRunning = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.MSAC;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement or not.
     *
     * @return true if quality scores are required, false otherwise.
     */
    @Override
    public boolean isQualityScoresRequired() {
        return false;
    }
}
