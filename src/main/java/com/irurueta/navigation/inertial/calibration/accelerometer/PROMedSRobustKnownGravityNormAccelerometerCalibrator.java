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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;

import java.util.List;

/**
 * Robustly estimates accelerometer biases, cross couplings and scaling factors
 * using a PROMedS algorithm to discard outliers.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single position
 * where gravity norm is known must be taken at 10 different unknown
 * orientations and zero velocity when common z-axis is assumed, otherwise at
 * least 13 measurements are required.
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias. Ideally, on a perfect accelerometer, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-truth specific force.
 * - w is measurement noise.
 */
public class PROMedSRobustKnownGravityNormAccelerometerCalibrator extends
        RobustKnownGravityNormAccelerometerCalibrator {

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
    public static final double DEFAULT_STOP_THRESHOLD = 1e-4;

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
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final List<StandardDeviationBodyKinematics> measurements) {
        super(measurements);
    }


    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final double[] initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final Matrix initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final Matrix initialBias, final Matrix initialMa) {
        super(initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
        super(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        super(groundTruthGravityNorm, measurements);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(final Acceleration groundTruthGravityNorm) {
        super(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        super(groundTruthGravityNorm, measurements);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(groundTruthGravityNorm, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm) {
        super(groundTruthGravityNorm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(groundTruthGravityNorm, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public PROMedSRobustKnownGravityNormAccelerometerCalibrator(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        super(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
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
     *                                  is smaller than minimum required samples
     *                                  (10 or 13).
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
     * and cross-coupling errors.
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
                        return PROMedSRobustKnownGravityNormAccelerometerCalibrator.this.isReady();
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
                                    PROMedSRobustKnownGravityNormAccelerometerCalibrator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                        if (listener != null) {
                            listener.onCalibrateProgressChange(
                                    PROMedSRobustKnownGravityNormAccelerometerCalibrator.this, progress);
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
     *                                  is smaller than the minimum required
     *                                  number of samples (10 or 13).
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null || qualityScores.length < getMinimumRequiredMeasurements()) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
