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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AccelerometerBiasUncertaintySource;
import com.irurueta.navigation.inertial.calibration.AccelerometerCalibrationSource;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate accelerometer biases,
 * cross couplings and scaling factors.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single position
 * where gravity norm is known must be taken at 10 different unknown
 * orientations and zero velocity when common z-axis
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
public abstract class RobustKnownGravityNormAccelerometerCalibrator implements
        AccelerometerNonLinearCalibrator, UnknownBiasNonLinearAccelerometerCalibrator, AccelerometerCalibrationSource,
        AccelerometerBiasUncertaintySource, OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator,
        QualityScoredAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            BaseGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL =
            BaseGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.LMEDS;

    /**
     * Indicates that result is refined by default.
     */
    public static final boolean DEFAULT_REFINE_RESULT = true;

    /**
     * Indicates that covariance is kept by default after refining result.
     */
    public static final boolean DEFAULT_KEEP_COVARIANCE = true;

    /**
     * Default amount of progress variation before notifying a change in estimation progress.
     * By default this is set to 5%.
     */
    public static final float DEFAULT_PROGRESS_DELTA = 0.05f;

    /**
     * Minimum allowed value for progress delta.
     */
    public static final float MIN_PROGRESS_DELTA = 0.0f;

    /**
     * Maximum allowed value for progress delta.
     */
    public static final float MAX_PROGRESS_DELTA = 1.0f;

    /**
     * Constant defining default confidence of the estimated result, which is
     * 99%. This means that with a probability of 99% estimation will be
     * accurate because chosen sub-samples will be inliers.
     */
    public static final double DEFAULT_CONFIDENCE = 0.99;

    /**
     * Default maximum allowed number of iterations.
     */
    public static final int DEFAULT_MAX_ITERATIONS = 5000;

    /**
     * Minimum allowed confidence value.
     */
    public static final double MIN_CONFIDENCE = 0.0;

    /**
     * Maximum allowed confidence value.
     */
    public static final double MAX_CONFIDENCE = 1.0;

    /**
     * Minimum allowed number of iterations.
     */
    public static final int MIN_ITERATIONS = 1;

    /**
     * Contains a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     */
    protected List<StandardDeviationBodyKinematics> measurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownGravityNormAccelerometerCalibratorListener listener;

    /**
     * Indicates whether calibrator is running.
     */
    protected boolean running;

    /**
     * Amount of progress variation before notifying a progress change during calibration.
     */
    protected float progressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%). The amount of confidence indicates the probability that the estimated
     * result is correct. Usually this value will be close to 1.0, but not exactly 1.0.
     */
    protected double confidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations. When the maximum number of iterations is
     * exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     */
    protected int maxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Data related to inliers found after calibration.
     */
    protected InliersData inliersData;

    /**
     * Indicates whether result must be refined using a non linear calibrator over
     * found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    protected boolean refineResult = DEFAULT_REFINE_RESULT;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int preliminarySubsetSize = MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double initialBiasX;

    /**
     * Initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double initialBiasY;

    /**
     * Initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double initialBiasZ;

    /**
     * Initial x scaling factor.
     */
    private double initialSx;

    /**
     * Initial y scaling factor.
     */
    private double initialSy;

    /**
     * Initial z scaling factor.
     */
    private double initialSz;

    /**
     * Initial x-y cross coupling error.
     */
    private double initialMxy;

    /**
     * Initial x-z cross coupling error.
     */
    private double initialMxz;

    /**
     * Initial y-x cross coupling error.
     */
    private double initialMyx;

    /**
     * Initial y-z cross coupling error.
     */
    private double initialMyz;

    /**
     * Initial z-x cross coupling error.
     */
    private double initialMzx;

    /**
     * Initial z-y cross coupling error.
     */
    private double initialMzy;

    /**
     * Ground truth gravity norm to be expected at location where measurements have been made,
     * expressed in meters per squared second (m/s^2).
     */
    protected Double groundTruthGravityNorm;

    /**
     * Estimated accelerometer biases for each IMU axis expressed in meter per squared
     * second (m/s^2).
     */
    private double[] estimatedBiases;

    /**
     * Estimated accelerometer scale factors and cross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix estimatedMa;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Estimated covariance of estimated position.
     * This is only available when result has been refined and covariance is kept.
     */
    private Matrix estimatedCovariance;

    /**
     * Estimated chi square value.
     */
    private double estimatedChiSq;

    /**
     * Estimated mean square error respect to provided measurements.
     */
    private double estimatedMse;

    /**
     * Inner calibrator to compute calibration for each subset of data or during
     * final refining.
     */
    private final KnownGravityNormAccelerometerCalibrator innerCalibrator =
            new KnownGravityNormAccelerometerCalibrator();

    /**
     * Contains 3x3 identify to be reused.
     */
    protected Matrix identity;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix tmp1;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix tmp2;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix tmp3;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix tmp4;

    /**
     * Constructor.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final List<StandardDeviationBodyKinematics> measurements) {
        this.measurements = measurements;
    }


    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final double[] initialBias) {
        try {
            setInitialBias(initialBias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final Matrix initialBias) {
        try {
            setInitialBias(initialBias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final Matrix initialBias, final Matrix initialMa) {
        this(initialBias);
        try {
            setInitialMa(initialMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        this(measurements);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(measurements);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, commonAxisUsed);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
        this.measurements = measurements;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, initialBias);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(groundTruthGravityNorm, measurements, initialBias);
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
        this.measurements = measurements;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, initialBias);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(groundTruthGravityNorm, measurements, initialBias);
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
        this.measurements = measurements;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, initialBias, initialMa);
        this.listener = listener;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        this(groundTruthGravityNorm, measurements, initialBias, initialMa);
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected RobustKnownGravityNormAccelerometerCalibrator(final Acceleration groundTruthGravityNorm) {
        this(convertAcceleration(groundTruthGravityNorm));
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        this(convertAcceleration(groundTruthGravityNorm), measurements);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, initialMa);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, initialBias, initialMa, listener);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias, initialMa);
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
    protected RobustKnownGravityNormAccelerometerCalibrator(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        this(convertAcceleration(groundTruthGravityNorm), measurements, commonAxisUsed, initialBias, initialMa,
                listener);
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solutions.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasX() {
        return initialBiasX;
    }

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasX(final double initialBiasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = initialBiasX;
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasY() {
        return initialBiasY;
    }

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasY(final double initialBiasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasY = initialBiasY;
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasZ() {
        return initialBiasZ;
    }

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasZ(final double initialBiasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasZ = initialBiasZ;
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasXAsAcceleration() {
        return new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasXAsAcceleration(final Acceleration result) {
        result.setValue(initialBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasX(final Acceleration initialBiasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = convertAcceleration(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasYAsAcceleration() {
        return new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasYAsAcceleration(final Acceleration result) {
        result.setValue(initialBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasY(final Acceleration initialBiasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasY = convertAcceleration(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasZAsAcceleration() {
        return new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasZAsAcceleration(final Acceleration result) {
        result.setValue(initialBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasZ(final Acceleration initialBiasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasZ = convertAcceleration(initialBiasZ);
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution
     * expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(final double initialBiasX, final double initialBiasY, final double initialBiasZ)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = initialBiasX;
        this.initialBiasY = initialBiasY;
        this.initialBiasZ = initialBiasZ;
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = convertAcceleration(initialBiasX);
        this.initialBiasY = convertAcceleration(initialBiasY);
        this.initialBiasZ = convertAcceleration(initialBiasZ);
    }

    /**
     * Gets initial bias coordinates of accelerometer used to find a solution.
     *
     * @return initial bias coordinates.
     */
    @Override
    public AccelerationTriad getInitialBiasAsTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Gets initial bias coordinates of accelerometer used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialBiasAsTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(initialBiasX, initialBiasY, initialBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets initial bias coordinates of accelerometer used to find a solution.
     *
     * @param initialBias initial bias coordinates to be set.
     */
    @Override
    public void setInitialBias(final AccelerationTriad initialBias) {
        initialBiasX = convertAcceleration(initialBias.getValueX(), initialBias.getUnit());
        initialBiasY = convertAcceleration(initialBias.getValueY(), initialBias.getUnit());
        initialBiasZ = convertAcceleration(initialBias.getValueZ(), initialBias.getUnit());
    }

    /**
     * Gets initial x scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x scaling factor.
     */
    @Override
    public double getInitialSx() {
        return initialSx;
    }

    /**
     * Sets initial x scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx initial x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSx(final double initialSx) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialSx = initialSx;
    }

    /**
     * Gets initial y scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y scaling factor.
     */
    @Override
    public double getInitialSy() {
        return initialSy;
    }

    /**
     * Sets initial y scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSy initial y scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSy(final double initialSy) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialSy = initialSy;
    }

    /**
     * Gets initial z scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z scaling factor.
     */
    @Override
    public double getInitialSz() {
        return initialSz;
    }

    /**
     * Sets initial z scaling factor.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSz initial z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSz(final double initialSz) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialSz = initialSz;
    }

    /**
     * Gets initial x-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-y cross coupling error.
     */
    @Override
    public double getInitialMxy() {
        return initialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxy initial x-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMxy(final double initialMxy) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMxy = initialMxy;
    }

    /**
     * Gets initial x-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial x-z cross coupling error.
     */
    @Override
    public double getInitialMxz() {
        return initialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxz initial x-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMxz(final double initialMxz) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMxz = initialMxz;
    }

    /**
     * Gets initial y-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-x cross coupling error.
     */
    @Override
    public double getInitialMyx() {
        return initialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMyx initial y-x cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMyx(final double initialMyx) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMyx = initialMyx;
    }

    /**
     * Gets initial y-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial y-z cross coupling error.
     */
    @Override
    public double getInitialMyz() {
        return initialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMyz initial y-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMyz(final double initialMyz) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMyz = initialMyz;
    }

    /**
     * Gets initial z-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-x cross coupling error.
     */
    @Override
    public double getInitialMzx() {
        return initialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMzx initial z-x cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMzx(final double initialMzx) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMzx = initialMzx;
    }

    /**
     * Gets initial z-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial z-y cross coupling error.
     */
    @Override
    public double getInitialMzy() {
        return initialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMzy initial z-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMzy(final double initialMzy) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialScalingFactors(final double initialSx, final double initialSy, final double initialSz)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialSx = initialSx;
        this.initialSy = initialSy;
        this.initialSz = initialSz;
    }

    /**
     * Sets initial cross coupling errors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialCrossCouplingErrors(
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialMxy = initialMxy;
        this.initialMxz = initialMxz;
        this.initialMyx = initialMyx;
        this.initialMyz = initialMyz;
        this.initialMzx = initialMzx;
        this.initialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors and cross coupling errors.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialScalingFactorsAndCrossCouplingErrors(
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        setInitialScalingFactors(initialSx, initialSy, initialSz);
        setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getInitialBias() {
        final var result = new double[BodyKinematics.COMPONENTS];
        getInitialBias(result);
        return result;
    }

    /**
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void getInitialBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = initialBiasX;
        result[1] = initialBiasY;
        result[2] = initialBiasZ;
    }

    /**
     * Sets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setInitialBias(final double[] initialBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (initialBias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        initialBiasX = initialBias[0];
        initialBiasY = initialBias[1];
        initialBiasZ = initialBias[2];
    }

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
     *
     * @return initial bias to be used to find a solution as a column matrix.
     */
    @Override
    public Matrix getInitialBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getInitialBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial bias to be used to find a solution as a column matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getInitialBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, initialBiasX);
        result.setElementAtIndex(1, initialBiasY);
        result.setElementAtIndex(2, initialBiasZ);
    }

    /**
     * Sets initial bias to be used to find a solution as an array.
     * This is only taken into account if non-linear preliminary solutions are used.
     * Values are expressed in meters per squared second (m/s^2).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setInitialBias(final Matrix initialBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (initialBias.getRows() != BodyKinematics.COMPONENTS || initialBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        initialBiasX = initialBias.getElementAtIndex(0);
        initialBiasY = initialBias.getElementAtIndex(1);
        initialBiasZ = initialBias.getElementAtIndex(2);
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    @Override
    public Matrix getInitialMa() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            getInitialMa(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void getInitialMa(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, initialSx);
        result.setElementAtIndex(1, initialMyx);
        result.setElementAtIndex(2, initialMzx);

        result.setElementAtIndex(3, initialMxy);
        result.setElementAtIndex(4, initialSy);
        result.setElementAtIndex(5, initialMzy);

        result.setElementAtIndex(6, initialMxz);
        result.setElementAtIndex(7, initialMyz);
        result.setElementAtIndex(8, initialSz);
    }

    /**
     * Sets initial scale factors and cross coupling errors matrix.
     * This is only taken into account if non-linear preliminary solutions are used.
     *
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setInitialMa(final Matrix initialMa) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (initialMa.getRows() != BodyKinematics.COMPONENTS || initialMa.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialSx = initialMa.getElementAtIndex(0);
        initialMyx = initialMa.getElementAtIndex(1);
        initialMzx = initialMa.getElementAtIndex(2);

        initialMxy = initialMa.getElementAtIndex(3);
        initialSy = initialMa.getElementAtIndex(4);
        initialMzy = initialMa.getElementAtIndex(5);

        initialMxz = initialMa.getElementAtIndex(6);
        initialMyz = initialMa.getElementAtIndex(7);
        initialSz = initialMa.getElementAtIndex(8);
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made,
     * expressed in meter per squared second (m/s^2).
     *
     * @return ground truth gravity norm or null.
     */
    public Double getGroundTruthGravityNorm() {
        return groundTruthGravityNorm;
    }

    /**
     * Sets ground truth gravity norm to be expected at location where
     * measurements have been made, expressed in meters per squared second
     * (m/s^2).
     *
     * @param groundTruthGravityNorm ground truth gravity norm or
     *                               null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthGravityNorm(final Double groundTruthGravityNorm) throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made.
     *
     * @return ground truth gravity norm or null.
     */
    public Acceleration getGroundTruthGravityNormAsAcceleration() {
        return groundTruthGravityNorm != null
                ? new Acceleration(groundTruthGravityNorm, AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets ground truth gravity norm to be expected at location where measurements have been made.
     *
     * @param result instance where result will be stored.
     * @return true if ground truth gravity norm has been defined, false if it is not available yet.
     */
    public boolean getGroundTruthGravityNormAsAcceleration(final Acceleration result) {
        if (groundTruthGravityNorm != null) {
            result.setValue(groundTruthGravityNorm);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets ground truth gravity norm to be expected at location where
     * measurements have been made.
     *
     * @param groundTruthGravityNorm ground truth gravity norm or null
     *                               if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthGravityNorm(final Acceleration groundTruthGravityNorm) throws LockedException {
        setGroundTruthGravityNorm(groundTruthGravityNorm != null ? convertAcceleration(groundTruthGravityNorm) : null);
    }

    /**
     * Gets a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     *
     * @return list of body kinematics measurements.
     */
    @Override
    public List<StandardDeviationBodyKinematics> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a list of body kinematics measurements taken at a
     * given position with different unknown orientations and containing the
     * standard deviations of accelerometer and gyroscope measurements.
     *
     * @param measurements list of body kinematics measurements.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(final List<StandardDeviationBodyKinematics> measurements) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.measurements = measurements;
    }

    /**
     * Indicates the type of measurement used by this calibrator.
     *
     * @return type of measurement used by this calibrator.
     */
    @Override
    public AccelerometerCalibratorMeasurementType getMeasurementType() {
        return AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS;
    }

    /**
     * Indicates whether this calibrator requires ordered measurements in a
     * list or not.
     *
     * @return true if measurements must be ordered, false otherwise.
     */
    @Override
    public boolean isOrderedMeasurementsRequired() {
        return true;
    }

    /**
     * Indicates whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer and gyroscope,
     * false otherwise.
     */
    @Override
    public boolean isCommonAxisUsed() {
        return commonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for accelerometer
     *                       and gyroscope, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setCommonAxisUsed(final boolean commonAxisUsed) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RobustKnownGravityNormAccelerometerCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownGravityNormAccelerometerCalibratorListener listener)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets minimum number of required measurements.
     *
     * @return minimum number of required measurements.
     */
    @Override
    public int getMinimumRequiredMeasurements() {
        return commonAxisUsed ? MINIMUM_MEASUREMENTS_COMMON_Z_AXIS : MINIMUM_MEASUREMENTS_GENERAL;
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return measurements != null && measurements.size() >= getMinimumRequiredMeasurements()
                && groundTruthGravityNorm != null;
    }

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return running;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * calibration.
     *
     * @return amount of progress variation before notifying a progress change during
     * calibration.
     */
    public float getProgressDelta() {
        return progressDelta;
    }

    /**
     * Sets amount of progress variation before notifying a progress change during
     * calibration.
     *
     * @param progressDelta amount of progress variation before notifying a progress
     *                      change during calibration.
     * @throws IllegalArgumentException if progress delta is less than zero or greater than 1.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setProgressDelta(final float progressDelta) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        this.progressDelta = progressDelta;
    }

    /**
     * Returns amount of confidence expressed as a value between 0.0 and 1.0
     * (which is equivalent to 100%). The amount of confidence indicates the probability
     * that the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     *
     * @return amount of confidence as a value between 0.0 and 1.0.
     */
    public double getConfidence() {
        return confidence;
    }

    /**
     * Sets amount of confidence expressed as a value between 0.0 and 1.0 (which is
     * equivalent to 100%). The amount of confidence indicates the probability that
     * the estimated result is correct. Usually this value will be close to 1.0, but
     * not exactly 1.0.
     *
     * @param confidence confidence to be set as a value between 0.0 and 1.0.
     * @throws IllegalArgumentException if provided value is not between 0.0 and 1.0.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setConfidence(final double confidence) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        this.confidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling calibrate(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return maxIterations;
    }

    /**
     * Sets maximum allowed number of iterations. When the maximum number of iterations
     * is exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     *
     * @param maxIterations maximum allowed number of iterations to be set.
     * @throws IllegalArgumentException if provided value is less than 1.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setMaxIterations(final int maxIterations) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        this.maxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return inliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return refineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if calibrator is currently running.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.refineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return keepCovariance;
    }

    /**
     * Specifies whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @param keepCovariance true if covariance must be kept after refining result,
     *                       false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setCovarianceKept(final boolean keepCovariance) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.keepCovariance = keepCovariance;
    }

    /**
     * Returns quality scores corresponding to each measurement.
     * The larger the score value the better the quality of the sample.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behavior.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return null;
    }

    /**
     * Sets quality scores corresponding to each measurement.
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each pair of
     *                      matched points.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    @Override
    public double[] getEstimatedBiases() {
        return estimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where estimated accelerometer biases will be stored.
     * @return true if result instance was updated, false otherwise (when estimation
     * is not yet available).
     */
    @Override
    public boolean getEstimatedBiases(final double[] result) {
        if (estimatedBiases != null) {
            System.arraycopy(estimatedBiases, 0, result, 0, estimatedBiases.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return column matrix containing x,y,z components of estimated accelerometer
     * biases
     */
    @Override
    public Matrix getEstimatedBiasesAsMatrix() {
        return estimatedBiases != null ? Matrix.newFromArray(estimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
    public boolean getEstimatedBiasesAsMatrix(final Matrix result) throws WrongSizeException {
        if (estimatedBiases != null) {
            result.fromArray(estimatedBiases);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFx() {
        return estimatedBiases != null ? estimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFy() {
        return estimatedBiases != null ? estimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias expressed in meters per
     * squared second (m/s^2).
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasFz() {
        return estimatedBiases != null ? estimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @return x coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFxAsAcceleration() {
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[0], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFxAsAcceleration(final Acceleration result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[0]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @return y coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFyAsAcceleration() {
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[1], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFyAsAcceleration(final Acceleration result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[1]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @return z coordinate of estimated accelerometer bias or null if not available.
     */
    @Override
    public Acceleration getEstimatedBiasFzAsAcceleration() {
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[2], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasFzAsAcceleration(final Acceleration result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[2]);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @return estimated accelerometer bias or null if not available.
     */
    @Override
    public AccelerationTriad getEstimatedBiasAsTriad() {
        return estimatedBiases != null
                ? new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                estimatedBiases[0], estimatedBiases[1], estimatedBiases[2])
                : null;
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated accelerometer bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedBiasAsTriad(final AccelerationTriad result) {
        if (estimatedBiases != null) {
            result.setValueCoordinatesAndUnit(estimatedBiases[0], estimatedBiases[1], estimatedBiases[2],
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated accelerometer scale factors and ross coupling errors.
     * This is the product of matrix Ta containing cross coupling errors and Ka
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated accelerometer scale factors and cross coupling errors, or null
     * if not available.
     */
    @Override
    public Matrix getEstimatedMa() {
        return estimatedMa;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return estimatedMa != null ? estimatedMa.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return estimatedMa != null ? estimatedMa.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return estimatedMa != null ? estimatedMa.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return estimatedMa != null ? estimatedMa.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return estimatedMa != null ? estimatedMa.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return estimatedMa != null ? estimatedMa.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return estimatedMa != null ? estimatedMa.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return estimatedMa != null ? estimatedMa.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return estimatedMa != null ? estimatedMa.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    @Override
    public double getEstimatedChiSq() {
        return estimatedChiSq;
    }

    /**
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    @Override
    public double getEstimatedMse() {
        return estimatedMse;
    }

    /**
     * Gets estimated covariance matrix for estimated calibration solution.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance matrix for estimated position.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return estimatedCovariance;
    }

    /**
     * Gets variance of estimated x coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated x coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFxVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated x coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFxStandardDeviation() {
        final var variance = getEstimatedBiasFxVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated x coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFxStandardDeviationAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasFxStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND) :
                null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFxStandardDeviationAsAcceleration(final Acceleration result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasFxStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated y coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFyVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated y coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFyStandardDeviation() {
        final var variance = getEstimatedBiasFyVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated y coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFyStandardDeviationAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasFyStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFyStandardDeviationAsAcceleration(final Acceleration result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasFyStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of accelerometer bias expressed in (m^2/s^4).
     *
     * @return variance of estimated z coordinate of accelerometer bias or null if not available.
     */
    public Double getEstimatedBiasFzVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     *
     * @return standard deviation of estimated z coordinate of accelerometer bias or null if not
     * available.
     */
    public Double getEstimatedBiasFzStandardDeviation() {
        final var variance = getEstimatedBiasFzVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias.
     *
     * @return standard deviation of estimated z coordinate of accelerometer bias or null if not
     * available.
     */
    public Acceleration getEstimatedBiasFzStandardDeviationAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasFzStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of accelerometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasFzStandardDeviationAsAcceleration(final Acceleration result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasFzStandardDeviation());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated accelerometer bias coordinates.
     *
     * @return standard deviation of estimated accelerometer bias coordinates.
     */
    public AccelerationTriad getEstimatedBiasStandardDeviation() {
        return estimatedCovariance != null
                ? new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                getEstimatedBiasFxStandardDeviation(),
                getEstimatedBiasFyStandardDeviation(),
                getEstimatedBiasFzStandardDeviation())
                : null;
    }

    /**
     * Gets standard deviation of estimated accelerometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of accelerometer bias was available, false
     * otherwise.
     */
    public boolean getEstimatedBiasStandardDeviation(final AccelerationTriad result) {
        if (estimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedBiasFxStandardDeviation(),
                    getEstimatedBiasFyStandardDeviation(),
                    getEstimatedBiasFzStandardDeviation(),
                    AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates expressed
     * in meters per squared second (m/s^2).
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null
     * if not available.
     */
    public Double getEstimatedBiasStandardDeviationAverage() {
        return estimatedCovariance != null ? (getEstimatedBiasFxStandardDeviation()
                + getEstimatedBiasFyStandardDeviation() + getEstimatedBiasFzStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null.
     */
    public Acceleration getEstimatedBiasStandardDeviationAverageAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasStandardDeviationAverage(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of accelerometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationAverageAsAcceleration(final Acceleration result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationAverage());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of accelerometer bias or null
     * if not available.
     */
    @Override
    public Double getEstimatedBiasStandardDeviationNorm() {
        return estimatedCovariance != null
                ? Math.sqrt(getEstimatedBiasFxVariance() + getEstimatedBiasFyVariance() + getEstimatedBiasFzVariance())
                : null;
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias.
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of accelerometer bias or null
     * if not available.
     */
    public Acceleration getEstimatedBiasStandardDeviationNormAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasStandardDeviationNorm(), AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
    }

    /**
     * Gets norm of estimated standard deviation of accelerometer bias coordinates.
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of accelerometer bias is
     * available, false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationNormAsAcceleration(final Acceleration result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationNorm());
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurements()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return preliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurements()}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #getMinimumRequiredMeasurements()}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinimumRequiredMeasurements()) {
            throw new IllegalArgumentException();
        }

        this.preliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator();
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator();
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator();
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator();
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator();
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final List<StandardDeviationBodyKinematics> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(measurements);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(measurements);
        };
    }


    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @param method      robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param initialBias initial bias to find a solution.
     * @param method      robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @param method      robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Matrix initialBias, final Matrix initialMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is
     *                                  smaller than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Double groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(groundTruthGravityNorm);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores          quality scores corresponding to each provided
     *                               measurement. The larger the score value the better
     *                               the quality of the sample.
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(
                    qualityScores, groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
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
     * @param method                 robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative or
     *                                  if provided quality scores length is smaller
     *                                  than 10 or 13 samples.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration groundTruthGravityNorm,
            final List<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case LMEDS -> new LMedSRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case MSAC -> new MSACRobustKnownGravityNormAccelerometerCalibrator(
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            case PROSAC -> new PROSACRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
            default -> new PROMedSRobustKnownGravityNormAccelerometerCalibrator(qualityScores,
                    groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final List<StandardDeviationBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }


    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(final double[] initialBias) {
        return create(initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param initialBias initial bias to find a solution.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(final Matrix initialBias) {
        return create(initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Matrix initialBias, final Matrix initialMa) {
        return create(initialBias, initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(final Double groundTruthGravityNorm) {
        return create(groundTruthGravityNorm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        return create(groundTruthGravityNorm, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        return create(groundTruthGravityNorm, measurements, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        return create(groundTruthGravityNorm, measurements, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        return create(groundTruthGravityNorm, measurements, initialBias, initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, initialMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Double groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(final Acceleration groundTruthGravityNorm) {
        return create(groundTruthGravityNorm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements) {
        return create(groundTruthGravityNorm, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           list of body kinematics measurements taken at a given position with
     *                               different unknown orientations and containing the standard deviations
     *                               of accelerometer and gyroscope measurements.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to be notified of events such as when estimation
     *                               starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        return create(groundTruthGravityNorm, measurements, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        return create(groundTruthGravityNorm, measurements, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        return create(groundTruthGravityNorm, measurements, initialBias, initialMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, initialBias, initialMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param groundTruthGravityNorm ground truth gravity norm.
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
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
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    public static RobustKnownGravityNormAccelerometerCalibrator create(
            final Acceleration groundTruthGravityNorm, final List<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final RobustKnownGravityNormAccelerometerCalibratorListener listener) {
        return create(groundTruthGravityNorm, measurements, commonAxisUsed, initialBias, initialMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of preliminary result respect a given measurement for current gravity norm.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationBodyKinematics measurement, final PreliminaryResult preliminaryResult) {

        try {
            // We know that measured specific force is:
            // fmeas = ba + (I + Ma) * ftrue

            // fmeas - ba = (I + Ma) * ftrue

            // ftrue = (I + Ma)^-1 * (fmeas - ba)

            // We know that ||ftrue|| should be equal to the gravity value at current Earth
            // position
            // ||ftrue|| = g ~ 9.81 m/s^2

            final var estBiases = preliminaryResult.estimatedBiases;
            final var estMa = preliminaryResult.estimatedMa;

            if (identity == null) {
                identity = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (tmp1 == null) {
                tmp1 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (tmp2 == null) {
                tmp2 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (tmp3 == null) {
                tmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            if (tmp4 == null) {
                tmp4 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            identity.add(estMa, tmp1);

            Utils.inverse(tmp1, tmp2);

            final var kinematics = measurement.getKinematics();
            final var fmeasX = kinematics.getFx();
            final var fmeasY = kinematics.getFy();
            final var fmeasZ = kinematics.getFz();

            final var bx = estBiases[0];
            final var by = estBiases[1];
            final var bz = estBiases[2];

            tmp3.setElementAtIndex(0, fmeasX - bx);
            tmp3.setElementAtIndex(1, fmeasY - by);
            tmp3.setElementAtIndex(2, fmeasZ - bz);

            tmp2.multiply(tmp3, tmp4);

            final var norm = Utils.normF(tmp4);
            final var diff = groundTruthGravityNorm - norm;

            return diff * diff;

        } catch (final AlgebraException e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Computes a preliminary solution for a subset of samples picked by a robust estimator.
     *
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions      list where estimated preliminary solution will be stored.
     */
    protected void computePreliminarySolutions(final int[] samplesIndices, final List<PreliminaryResult> solutions) {

        final var meas = new ArrayList<StandardDeviationBodyKinematics>();

        for (final var samplesIndex : samplesIndices) {
            meas.add(measurements.get(samplesIndex));
        }

        try {
            final var result = new PreliminaryResult();
            result.estimatedBiases = getInitialBias();
            result.estimatedMa = getInitialMa();

            innerCalibrator.setInitialBias(result.estimatedBiases);
            innerCalibrator.setInitialMa(result.estimatedMa);
            innerCalibrator.setCommonAxisUsed(commonAxisUsed);
            innerCalibrator.setGroundTruthGravityNorm(groundTruthGravityNorm);
            innerCalibrator.setMeasurements(meas);
            innerCalibrator.calibrate();

            innerCalibrator.getEstimatedBiases(result.estimatedBiases);
            result.estimatedMa = innerCalibrator.getEstimatedMa();

            if (keepCovariance) {
                result.covariance = innerCalibrator.getEstimatedCovariance();
            } else {
                result.covariance = null;
            }

            result.estimatedMse = innerCalibrator.getEstimatedMse();
            result.estimatedChiSq = innerCalibrator.getEstimatedChiSq();

            solutions.add(result);

        } catch (final LockedException | CalibrationException | NotReadyException e) {
            solutions.clear();
        }
    }

    /**
     * Attempts to refine calibration parameters if refinement is requested.
     * This method returns a refined solution or provided input if refinement is not
     * requested or has failed.
     * If refinement is enabled and it is requested to keep covariance, this method
     * will also keep covariance of refined position.
     *
     * @param preliminaryResult a preliminary result.
     */
    protected void attemptRefine(final PreliminaryResult preliminaryResult) {
        if (refineResult && inliersData != null) {
            final var inliers = inliersData.getInliers();
            final var nSamples = measurements.size();

            final var inlierMeasurements = new ArrayList<StandardDeviationBodyKinematics>();
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(measurements.get(i));
                }
            }

            try {
                innerCalibrator.setInitialBias(preliminaryResult.estimatedBiases);
                innerCalibrator.setInitialMa(preliminaryResult.estimatedMa);
                innerCalibrator.setCommonAxisUsed(commonAxisUsed);
                innerCalibrator.setGroundTruthGravityNorm(groundTruthGravityNorm);
                innerCalibrator.setMeasurements(inlierMeasurements);
                innerCalibrator.calibrate();

                estimatedBiases = innerCalibrator.getEstimatedBiases();
                estimatedMa = innerCalibrator.getEstimatedMa();
                estimatedMse = innerCalibrator.getEstimatedMse();
                estimatedChiSq = innerCalibrator.getEstimatedChiSq();

                if (keepCovariance) {
                    estimatedCovariance = innerCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }
            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = preliminaryResult.covariance;
                estimatedBiases = preliminaryResult.estimatedBiases;
                estimatedMa = preliminaryResult.estimatedMa;
                estimatedMse = preliminaryResult.estimatedMse;
                estimatedChiSq = preliminaryResult.estimatedChiSq;
            }
        } else {
            estimatedCovariance = preliminaryResult.covariance;
            estimatedBiases = preliminaryResult.estimatedBiases;
            estimatedMa = preliminaryResult.estimatedMa;
            estimatedMse = preliminaryResult.estimatedMse;
            estimatedChiSq = preliminaryResult.estimatedChiSq;
        }
    }

    /**
     * Internally sets ground truth gravity norm to be expected at location where
     * measurements have been made, expressed in meters per squared second
     * (m/s^2).
     *
     * @param groundTruthGravityNorm ground truth gravity norm or null if
     *                               undefined.
     * @throws IllegalArgumentException if provided value is negative.
     */
    private void internalSetGroundTruthGravityNorm(final Double groundTruthGravityNorm) {
        if (groundTruthGravityNorm != null && groundTruthGravityNorm < 0.0) {
            throw new IllegalArgumentException();
        }
        this.groundTruthGravityNorm = groundTruthGravityNorm;
    }

    /**
     * Converts acceleration value and unit to meters per squared second.
     *
     * @param value acceleration value.
     * @param unit  unit of acceleration value.
     * @return converted value.
     */
    private static double convertAcceleration(final double value, final AccelerationUnit unit) {
        return AccelerationConverter.convert(value, unit, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(), acceleration.getUnit());
    }

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
        /**
         * Estimated accelerometer biases for each IMU axis expressed in meter per squared
         * second (m/s^2).
         */
        private double[] estimatedBiases;

        /**
         * Estimated accelerometer scale factors and cross coupling errors.
         * This is the product of matrix Ta containing cross coupling errors and Ka
         * containing scaling factors.
         * So tat:
         * <pre>
         *     Ma = [sx    mxy  mxz] = Ta*Ka
         *          [myx   sy   myz]
         *          [mzx   mzy  sz ]
         * </pre>
         * Where:
         * <pre>
         *     Ka = [sx 0   0 ]
         *          [0  sy  0 ]
         *          [0  0   sz]
         * </pre>
         * and
         * <pre>
         *     Ta = [1          -alphaXy    alphaXz ]
         *          [alphaYx    1           -alphaYz]
         *          [-alphaZx   alphaZy     1       ]
         * </pre>
         * Hence:
         * <pre>
         *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
         *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
         *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
         * </pre>
         * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
         * are considered to be zero if the accelerometer z-axis is assumed to be the same
         * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
         * becomes upper diagonal:
         * <pre>
         *     Ma = [sx    mxy  mxz]
         *          [0     sy   myz]
         *          [0     0    sz ]
         * </pre>
         * Values of this matrix are unit-less.
         */
        private Matrix estimatedMa;

        /**
         * Estimated covariance matrix.
         */
        private Matrix covariance;

        /**
         * Estimated MSE (Mean Square Error).
         */
        private double estimatedMse;

        /**
         * Estimated chi square value.
         */
        private double estimatedChiSq;
    }
}
