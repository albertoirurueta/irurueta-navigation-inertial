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
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * Robustly estimates accelerometer biases, cross couplings and scaling factors
 * using a PROSAC algorithm to discard outliers.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single known position must
 * be taken at 10 different unknown orientations and zero velocity when common z-axis
 * is assumed, otherwise at least 13 measurements are required.
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
public class PROSACRobustKnownPositionAccelerometerCalibrator extends RobustKnownPositionAccelerometerCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-2;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and
     * distances provided for each sample.
     */
    private double threshold = DEFAULT_THRESHOLD;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(final List<StandardDeviationBodyKinematics> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(final boolean commonAxisUsed) {
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
    public PROSACRobustKnownPositionAccelerometerCalibrator(final double[] initialBias) {
        super(initialBias);
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(final Matrix initialBias) {
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
    public PROSACRobustKnownPositionAccelerometerCalibrator(final Matrix initialBias, final Matrix initialMa) {
        super(initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(final ECEFPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(position, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(position, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body kinematics measures have been taken.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(final NEDPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements list of body kinematics measurements taken at a given position with
     *                     different unknown orientations and containing the standard deviations
     *                     of accelerometer and gyroscope measurements.
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        super(position, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        super(position, measurements, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position     position where body kinematics measures have been taken.
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
    }

    /**
     * Constructor.
     *
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 13 samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 13 samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 10 samples are
     *                       required, otherwise 13.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 10 samples are
     *                       required, otherwise 13.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial accelerometer bias to be used to find a solution.
     *                      This must have length 3 and is expressed in meters per
     *                      squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias) {
        super(position, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial accelerometer bias to be used to find a solution.
     *                      This must have length 3 and is expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias) {
        super(position, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 13 samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements) {
        super(position, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  list of body kinematics measurements taken at a given position with
     *                      different unknown orientations and containing the standard deviations
     *                      of accelerometer and gyroscope measurements.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 13 samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 10 samples are
     *                       required, otherwise 13.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   list of body kinematics measurements taken at a given position with
     *                       different unknown orientations and containing the standard deviations
     *                       of accelerometer and gyroscope measurements.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope. If true 10 samples are
     *                       required, otherwise 13.
     * @param listener       listener to be notified of events such as when estimation
     *                       starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than the minimum number of
     *                                  required samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial accelerometer bias to be used to find a solution.
     *                      This must have length 3 and is expressed in meters per
     *                      squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias) {
        super(position, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial accelerometer bias to be used to find a solution.
     *                      This must have length 3 and is expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias) {
        super(position, measurements, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or if
     *                                  quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        super(position, measurements, commonAxisUsed, initialBias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMa) {
        super(position, measurements, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body kinematics measures have been taken.
     * @param measurements  collection of body kinematics measurements with standard
     *                      deviations taken at the same position with zero velocity
     *                      and unknown different orientations.
     * @param initialBias   initial bias to find a solution.
     * @param initialMa     initial scale factors and cross coupling errors matrix.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if quality scores array is smaller than 13
     *                                  samples.
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body kinematics measures have been taken.
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or if provided quality scores length is
     *                                  smaller than the minimum number of required
     *                                  samples (10 or 13).
     */
    public PROSACRobustKnownPositionAccelerometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownPositionAccelerometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, initialBias, initialMa, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
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
     * Indicates whether calibrator is ready to find a solution.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == measurements.size();
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return computeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.computeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return computeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.computeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates accelerometer calibration parameters containing bias, scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (running) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        gravityNorm = computeGravityNorm();

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<PreliminaryResult>() {
            @Override
            public double[] getQualityScores() {
                return qualityScores;
            }

            @Override
            public double getThreshold() {
                return threshold;
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
                return PROSACRobustKnownPositionAccelerometerCalibrator.this.isReady();
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
                            PROSACRobustKnownPositionAccelerometerCalibrator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                if (listener != null) {
                    listener.onCalibrateProgressChange(
                            PROSACRobustKnownPositionAccelerometerCalibrator.this, progress);
                }
            }
        });

        try {
            running = true;

            if (listener != null) {
                listener.onCalibrateStart(this);
            }

            inliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(computeAndKeepInliers || refineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(computeAndKeepResiduals || refineResult);
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
        return RobustEstimatorMethod.PROSAC;
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
