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
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates magnetometer hard-iron biases, cross couplings and
 * scaling factors using PROMedS algorithm.
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
public class PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator extends
        RobustKnownMagneticFluxDensityNormMagnetometerCalibrator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-9;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double stopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(final double[] initialHardIron) {
        super(initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(final Matrix initialHardIron) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
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
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final boolean commonAxisUsed) {
        super(commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final double[] initialHardIron) {
        super(initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Matrix initialHardIron) {
        super(initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Matrix initialHardIron, final Matrix initialMm) {
        super(initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        super(measurements, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
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
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        super(measurements, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        super(measurements, commonAxisUsed, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
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
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.*
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
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        super(measurements, initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
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
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, initialHardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
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
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
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
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm) {
        super(groundTruthMagneticFluxDensityNorm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(groundTruthMagneticFluxDensityNorm, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final boolean commonAxisUsed) {
        super(groundTruthMagneticFluxDensityNorm, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3, or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1, or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm, final Matrix initialHardIron,
            final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
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
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        super(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return stopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.stopThreshold = stopThreshold;
    }

    /**
     * Returns quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @param qualityScores quality scores corresponding to each sample.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == measurements.size();
    }

    /**
     * Estimates magnetometer calibration parameters containing hard-iron
     * bias and soft-iron scale factors and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (running) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROMedSRobustEstimator<>(
                new PROMedSRobustEstimatorListener<PreliminaryResult>() {
                    @Override
                    public double[] getQualityScores() {
                        return qualityScores;
                    }

                    @Override
                    public double getThreshold() {
                        return stopThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return measurements.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return preliminarySubsetSize;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices, final List<PreliminaryResult> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(final PreliminaryResult currentEstimation, final int i) {
                        return computeError(measurements.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.this.isReady();
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
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onCalibrateNextIteration(
                                    PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.this,
                                    iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                        if (listener != null) {
                            listener.onCalibrateProgressChange(
                                    PROMedSRobustKnownMagneticFluxDensityNormMagnetometerCalibrator.this,
                                    progress);
                        }
                    }
                });

        try {
            running = true;

            if (listener != null) {
                listener.onCalibrateStart(this);
            }

            inliersData = null;

            innerEstimator.setUseInlierThresholds(true);
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            final var preliminaryResult = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

            if (listener != null) {
                listener.onCalibrateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (final RobustEstimatorException e) {
            throw new CalibrationException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMEDS;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement or not.
     *
     * @return true if quality scores are required, false otherwise.
     */
    @Override
    public boolean isQualityScoresRequired() {
        return true;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null || qualityScores.length < MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
