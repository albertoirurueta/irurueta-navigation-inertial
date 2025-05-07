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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate accelerometer
 * cross couplings and scaling factors.
 * This estimator assumes that biases are known.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, accelerometer samples must be obtained at 4
 * different positions, orientations and velocities (although typically velocities are
 * always zero).
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
public abstract class RobustKnownBiasAndFrameAccelerometerCalibrator implements
        AccelerometerNonLinearCalibrator, KnownBiasAccelerometerCalibrator,
        OrderedStandardDeviationFrameBodyKinematicsAccelerometerCalibrator,
        QualityScoredAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 4;

    /**
     * Indicates that by default a linear calibrator is used for preliminary solution estimation.
     * The result obtained on each preliminary solution might be later refined.
     */
    public static final boolean DEFAULT_USE_LINEAR_CALIBRATOR = true;

    /**
     * Indicates that by default preliminary solutions are refined.
     */
    public static final boolean DEFAULT_REFINE_PRELIMINARY_SOLUTIONS = false;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD = RobustEstimatorMethod.LMEDS;

    /**
     * Indicates that result is refined by default using a non-linear calibrator
     * (which uses a Levenberg-Marquardt fitter).
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
     * Contains a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     */
    protected List<StandardDeviationFrameBodyKinematics> measurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibratorListener listener;

    /**
     * Indicates whether estimator is running.
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
    protected int preliminarySubsetSize = MINIMUM_MEASUREMENTS;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasX;

    /**
     * Known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasY;

    /**
     * Known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasZ;

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
     * Indicates whether a linear calibrator is used or not for preliminary
     * solutions.
     */
    private boolean useLinearCalibrator = DEFAULT_USE_LINEAR_CALIBRATOR;

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution
     * is found.
     */
    private boolean refinePreliminarySolutions = DEFAULT_REFINE_PRELIMINARY_SOLUTIONS;

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
     * A linear least squares calibrator.
     */
    private final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator linearCalibrator =
            new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

    /**
     * A non-linear least squares calibrator.
     */
    private final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator nonLinearCalibrator =
            new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

    /**
     * Constructor.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(measurements);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(measurements);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(final double[] bias) {
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias) {
        this(measurements);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(final double[] bias, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(final Matrix bias) {
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias) {
        this(measurements);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(final Matrix bias, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetBias(bias);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    protected RobustKnownBiasAndFrameAccelerometerCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetBias(bias);
    }

    /**
     * Gets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x coordinate of accelerometer bias.
     */
    @Override
    public double getBiasX() {
        return biasX;
    }

    /**
     * Sets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.biasX = biasX;
    }

    /**
     * Gets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y coordinate of accelerometer bias.
     */
    @Override
    public double getBiasY() {
        return biasY;
    }

    /**
     * Sets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasY = biasY;
    }

    /**
     * Gets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z coordinate of accelerometer bias.
     */
    @Override
    public double getBiasZ() {
        return biasZ;
    }

    /**
     * Sets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasZ = biasZ;
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @return x coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(biasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known x coordinate of accelerometer bias.
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final Acceleration biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasX = convertAcceleration(biasX);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @return y coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(biasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known y coordinate of accelerometer bias.
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final Acceleration biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasY = convertAcceleration(biasY);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @return z coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(biasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known z coordinate of accelerometer bias.
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final Acceleration biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasZ = convertAcceleration(biasZ);
    }

    /**
     * Sets known accelerometer bias coordinates expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final double biasX, final double biasY, final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Sets known accelerometer bias coordinates.
     *
     * @param biasX x coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        internalSetBiasCoordinates(biasX, biasY, biasZ);
    }

    /**
     * Gets known accelerometer bias.
     *
     * @return known accelerometer bias.
     */
    @Override
    public AccelerationTriad getBiasAsTriad() {
        return new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, biasX, biasY, biasZ);
    }

    /**
     * Gets known accelerometer bias.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getBiasAsTriad(final AccelerationTriad result) {
        result.setValueCoordinatesAndUnit(biasX, biasY, biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known accelerometer bias.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBias(final AccelerationTriad bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        biasX = convertAcceleration(bias.getValueX(), bias.getUnit());
        biasY = convertAcceleration(bias.getValueY(), bias.getUnit());
        biasZ = convertAcceleration(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @return array containing coordinates of known bias.
     */
    @Override
    public double[] getBias() {
        final var result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void getBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = biasX;
        result[1] = biasY;
        result[2] = biasZ;
    }

    /**
     * Sets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setBias(final double[] bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        internalSetBias(bias);
    }

    /**
     * Gets known accelerometer bias as a column matrix.
     *
     * @return known accelerometer bias as a column matrix.
     */
    @Override
    public Matrix getBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known accelerometer bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, biasX);
        result.setElementAtIndex(1, biasY);
        result.setElementAtIndex(2, biasZ);
    }

    /**
     * Sets known accelerometer bias as a column matrix.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException          if calibrator is currently running
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setBias(final Matrix bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        internalSetBias(bias);
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
    public void setInitialScalingFactors(
            final double initialSx, final double initialSy, final double initialSz) throws LockedException {
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
     * Gets a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate the a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public List<StandardDeviationFrameBodyKinematics> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a list of body kinematics measurements taken at different
     * frames (positions, orientations and velocities) and containing the standard
     * deviations of accelerometer and gyroscope measurements.
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate the a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @param measurements collection of body kinematics measurements taken at different
     *                     frames (positions, orientations and velocities).
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(final List<StandardDeviationFrameBodyKinematics> measurements) throws LockedException {
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
        return AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS;
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
    public RobustKnownBiasAndFrameAccelerometerCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener)
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
        return MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return measurements != null && measurements.size() >= MINIMUM_MEASUREMENTS;
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
     * Indicates whether a linear calibrator is used or not for preliminary
     * solutions.
     *
     * @return indicates whether a linear calibrator is used or not for
     * preliminary solutions.
     */
    public boolean isLinearCalibratorUsed() {
        return useLinearCalibrator;
    }

    /**
     * Specifies whether a linear calibrator is used or not for preliminary
     * solutions.
     *
     * @param linearCalibratorUsed indicates whether a linear calibrator is used
     *                             or not for preliminary solutions.
     * @throws LockedException if calibrator is currently running.
     */
    public void setLinearCalibratorUsed(final boolean linearCalibratorUsed) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        useLinearCalibrator = linearCalibratorUsed;
    }

    /**
     * Indicates whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @return true if preliminary solutions must be refined after an initial linear solution, false
     * otherwise.
     */
    public boolean isPreliminarySolutionRefined() {
        return refinePreliminarySolutions;
    }

    /**
     * Specifies whether preliminary solutions must be refined after an initial linear solution is found.
     * If no initial solution is found using a linear solver, a non linear solver will be
     * used regardless of this value using an average solution as the initial value to be
     * refined.
     *
     * @param preliminarySolutionRefined true if preliminary solutions must be refined after an
     *                                   initial linear solution, false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPreliminarySolutionRefined(final boolean preliminarySolutionRefined) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        refinePreliminarySolutions = preliminarySolutionRefined;
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
     * This is only available when result has been refined and covariance
     * is kept.
     *
     * @return estimated covariance matrix for estimated calibration parameters.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return estimatedCovariance;
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
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return preliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_MEASUREMENTS}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < MINIMUM_MEASUREMENTS) {
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
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator();
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator();
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator();
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator();
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator();
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
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(listener);
        };
    }


    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
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
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX  known x coordinate of accelerometer bias expressed in meters per
     *               squared second (m/s^2).
     * @param biasY  known y coordinate of accelerometer bias expressed in meters per
     *               squared second (m/s^2).
     * @param biasZ  known z coordinate of accelerometer bias expressed in meters per
     *               squared second (m/s^2).
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC ->
                    new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX  known x coordinate of accelerometer bias.
     * @param biasY  known y coordinate of accelerometer bias.
     * @param biasZ  known z coordinate of accelerometer bias.
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias   known accelerometer bias.
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias   known accelerometer bias.
     * @param method robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasY         known y coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param biasZ         known z coordinate of accelerometer bias expressed in meters per
     *                      squared second (m/s^2).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores,
                    biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of accelerometer bias.
     * @param biasY         known y coordinate of accelerometer bias.
     * @param biasZ         known z coordinate of accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(biasX, biasY, biasZ, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, biasX, biasY, biasZ,
                    commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements,
                    biasX, biasY, biasZ, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double[] bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double[] bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias,
                    commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Matrix bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known accelerometer bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Matrix bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known accelerometer bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Matrix bias, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias, commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(bias, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, bias,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust accelerometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided quality scores length is smaller than
     *                                  4 samples.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case LMEDS -> new LMedSRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case MSAC -> new MSACRobustKnownBiasAndFrameAccelerometerCalibrator(measurements, bias, commonAxisUsed,
                    listener);
            case PROSAC -> new PROSACRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed, listener);
            default -> new PROMedSRobustKnownBiasAndFrameAccelerometerCalibrator(qualityScores, measurements, bias,
                    commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ) {
        return create(biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        return create(measurements, biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        return create(biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(biasX, biasY, biasZ, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in meters per
     *                       squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        return create(biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        return create(measurements, biasX, biasY, biasZ, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        return create(biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(biasX, biasY, biasZ, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, biasX, biasY, biasZ, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(final double[] bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias) {
        return create(measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final boolean commonAxisUsed) {
        return create(bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed) {
        return create(measurements, bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(final Matrix bias) {
        return create(bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias) {
        return create(measurements, bias, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, bias, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final boolean commonAxisUsed) {
        return create(bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed) {
        return create(measurements, bias, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust accelerometer calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public static RobustKnownBiasAndFrameAccelerometerCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameAccelerometerCalibratorListener listener) {
        return create(measurements, bias, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement   a measurement.
     * @param preliminaryMa a preliminary result.
     * @return computed error.
     */
    protected double computeError(final StandardDeviationFrameBodyKinematics measurement, final Matrix preliminaryMa) {
        // We know that measured specific force is:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        // [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        // [fmeasy]   [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        // [fmeasz]   [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        final var measuredKinematics = measurement.getKinematics();
        final var ecefFrame = measurement.getFrame();
        final var previousEcefFrame = measurement.getPreviousFrame();
        final var timeInterval = measurement.getTimeInterval();

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                previousEcefFrame);

        final var fMeasX1 = measuredKinematics.getFx();
        final var fMeasY1 = measuredKinematics.getFy();
        final var fMeasZ1 = measuredKinematics.getFz();

        final var fTrueX = expectedKinematics.getFx();
        final var fTrueY = expectedKinematics.getFy();
        final var fTrueZ = expectedKinematics.getFz();

        try {
            final var m = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            m.add(preliminaryMa);

            final var ftrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            ftrue.setElementAtIndex(0, fTrueX);
            ftrue.setElementAtIndex(1, fTrueY);
            ftrue.setElementAtIndex(2, fTrueZ);

            m.multiply(ftrue);

            final var fMeasX2 = biasX + m.getElementAtIndex(0);
            final var fMeasY2 = biasY + m.getElementAtIndex(1);
            final var fMeasZ2 = biasZ + m.getElementAtIndex(2);

            final var diffX = fMeasX2 - fMeasX1;
            final var diffY = fMeasY2 - fMeasY1;
            final var diffZ = fMeasZ2 - fMeasZ1;

            return Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        } catch (final WrongSizeException e) {
            return Double.MAX_VALUE;
        }
    }

    /**
     * Computes a preliminary solution for a subset of samples picked by a robust estimator.
     *
     * @param samplesIndices indices of samples picked by the robust estimator.
     * @param solutions      list where estimated preliminary solution will be stored.
     */
    protected void computePreliminarySolutions(final int[] samplesIndices, final List<Matrix> solutions) {

        final var meas = new ArrayList<StandardDeviationFrameBodyKinematics>();

        for (var samplesIndex : samplesIndices) {
            meas.add(measurements.get(samplesIndex));
        }

        try {
            var result = getInitialMa();

            if (useLinearCalibrator) {
                linearCalibrator.setCommonAxisUsed(commonAxisUsed);
                linearCalibrator.setMeasurements(meas);
                linearCalibrator.setBiasCoordinates(biasX, biasY, biasZ);
                linearCalibrator.calibrate();

                result = linearCalibrator.getEstimatedMa();
            }

            if (refinePreliminarySolutions) {
                nonLinearCalibrator.setInitialMa(result);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(meas);
                nonLinearCalibrator.setBiasCoordinates(biasX, biasY, biasZ);
                nonLinearCalibrator.calibrate();

                result = nonLinearCalibrator.getEstimatedMa();
            }

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
    protected void attemptRefine(final Matrix preliminaryResult) {
        if (refineResult && inliersData != null) {
            final var inliers = inliersData.getInliers();
            final var nSamples = measurements.size();

            final var inlierMeasurements = new ArrayList<StandardDeviationFrameBodyKinematics>();
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(measurements.get(i));
                }
            }

            try {
                nonLinearCalibrator.setInitialMa(preliminaryResult);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(inlierMeasurements);
                nonLinearCalibrator.setBiasCoordinates(biasX, biasY, biasZ);
                nonLinearCalibrator.calibrate();

                estimatedMa = nonLinearCalibrator.getEstimatedMa();
                estimatedMse = nonLinearCalibrator.getEstimatedMse();
                estimatedChiSq = nonLinearCalibrator.getEstimatedChiSq();

                if (keepCovariance) {
                    estimatedCovariance = nonLinearCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = null;
                estimatedMa = preliminaryResult;
                estimatedMse = 0.0;
                estimatedChiSq = 0.0;
            }
        } else {
            estimatedCovariance = null;
            estimatedMa = preliminaryResult;
            estimatedMse = 0.0;
            estimatedChiSq = 0.0;
        }
    }

    /**
     * Internally sets bias coordinates.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     */
    private void internalSetBiasCoordinates(final double biasX, final double biasY, final double biasZ) {
        this.biasX = biasX;
        this.biasY = biasY;
        this.biasZ = biasZ;
    }

    /**
     * Internally sets bias coordinates.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     */
    private void internalSetBiasCoordinates(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this.biasX = convertAcceleration(biasX);
        this.biasY = convertAcceleration(biasY);
        this.biasZ = convertAcceleration(biasZ);
    }

    /**
     * Internally sets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    private void internalSetBias(final double[] bias) {
        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        biasX = bias[0];
        biasY = bias[1];
        biasZ = bias[2];
    }

    /**
     * Internally sets known accelerometer bias as a column matrix.
     * Matrix values are expressed in meters per squared second (m/s^2).
     *
     * @param bias accelerometer bias to be set.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    private void internalSetBias(final Matrix bias) {
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        biasX = bias.getElementAtIndex(0);
        biasY = bias.getElementAtIndex(1);
        biasZ = bias.getElementAtIndex(2);
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
}
