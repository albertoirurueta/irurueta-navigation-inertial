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
package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationFixer;
import com.irurueta.navigation.inertial.calibration.AngularRateFixer;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.List;

/**
 * This is an abstract class to robustly estimate gyroscope
 * cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator assumes that the IMU is at a more or less fixed location on
 * Earth, and evaluates sequences of measured body kinematics to perform
 * calibration for unknown orientations on those provided sequences.
 * Each provided sequence will be preceded by a static period where mean
 * specific force will be measured to determine gravity (and hence partial
 * body attitude).
 * <p>
 * Measured gyroscope angular rates is assumed to follow the model shown below:
 * <pre>
 *     Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w
 * </pre>
 * Where:
 * - Ωmeas is the measured gyroscope angular rates. This is a 3x1 vector.
 * - bg is the gyroscope bias. Ideally, on a perfect gyroscope, this should be a
 * 3x1 zero vector.
 * - I is the 3x3 identity matrix.
 * - Mg is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect gyroscope, this should be a 3x3 zero matrix.
 * - Ωtrue is ground-truth gyroscope angular rates.
 * - Gg is the G-dependent cross biases introduced by the specific forces sensed
 * by the accelerometer. Ideally, on a perfect gyroscope, this should be a 3x3
 * zero matrix.
 * - ftrue is ground-truth specific force. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
public abstract class RobustKnownBiasEasyGyroscopeCalibrator implements
        GyroscopeNonLinearCalibrator, KnownBiasGyroscopeCalibrator, OrderedBodyKinematicsSequenceGyroscopeCalibrator,
        QualityScoredGyroscopeCalibrator, AccelerometerDependentGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = true;

    /**
     * Indicates that by default G-dependent cross biases introduced
     * by the accelerometer on the gyroscope are estimated.
     */
    public static final boolean DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES = true;

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
     * Known x-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasX;

    /**
     * Known y-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasY;

    /**
     * Known z-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double mAccelerometerBiasZ;

    /**
     * Known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSx;

    /**
     * Known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSy;

    /**
     * Known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerSz;

    /**
     * Known accelerometer x-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMxy;

    /**
     * Know accelerometer x-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMxz;

    /**
     * Known accelerometer y-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMyx;

    /**
     * Known accelerometer y-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMyz;

    /**
     * Known accelerometer z-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMzx;

    /**
     * Known accelerometer z-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double mAccelerometerMzy;

    /**
     * X-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double mBiasX;

    /**
     * Y-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double mBiasY;

    /**
     * Z-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double mBiasZ;

    /**
     * Initial gyroscope x scaling factor.
     */
    private double mInitialSx;

    /**
     * Initial gyroscope y scaling factor.
     */
    private double mInitialSy;

    /**
     * Initial gyroscope z scaling factor.
     */
    private double mInitialSz;

    /**
     * Initial gyroscope x-y cross coupling error.
     */
    private double mInitialMxy;

    /**
     * Initial gyroscope x-z cross coupling error.
     */
    private double mInitialMxz;

    /**
     * Initial gyroscope y-x cross coupling error.
     */
    private double mInitialMyx;

    /**
     * Initial gyroscope y-z cross coupling error.
     */
    private double mInitialMyz;

    /**
     * Initial gyroscope z-x cross coupling error.
     */
    private double mInitialMzx;

    /**
     * Initial gyroscope z-y cross coupling error.
     */
    private double mInitialMzy;

    /**
     * Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     */
    private Matrix mInitialGg;

    /**
     * Contains a collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     */
    protected List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mSequences;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * This flag indicates whether G-dependent cross biases are being
     * estimated or not.
     * When enabled, this adds 9 variables from Gg matrix.
     */
    private boolean mEstimateGDependentCrossBiases = DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasEasyGyroscopeCalibratorListener mListener;

    /**
     * Estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix mEstimatedMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix mEstimatedGg;

    /**
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix mEstimatedCovariance;

    /**
     * Estimated chi square value.
     */
    private double mEstimatedChiSq;

    /**
     * Estimated mean square error respect to provided measurements.
     */
    private double mEstimatedMse;

    /**
     * Indicates whether calibrator is running.
     */
    protected boolean mRunning;

    /**
     * Amount of progress variation before notifying a progress change during calibration.
     */
    protected float mProgressDelta = DEFAULT_PROGRESS_DELTA;

    /**
     * Amount of confidence expressed as a value between 0.0 and 1.0 (which is equivalent
     * to 100%). The amount of confidence indicates the probability that the estimated
     * result is correct. Usually this value will be close to 1.0, but not exactly 1.0.
     */
    protected double mConfidence = DEFAULT_CONFIDENCE;

    /**
     * Maximum allowed number of iterations. When the maximum number of iterations is
     * exceeded, result will not be available, however an approximate result will be
     * available for retrieval.
     */
    protected int mMaxIterations = DEFAULT_MAX_ITERATIONS;

    /**
     * Data related to inliers found after calibration.
     */
    protected InliersData mInliersData;

    /**
     * Indicates whether result must be refined using a non linear calibrator over
     * found inliers.
     * If true, inliers will be computed and kept in any implementation regardless of the
     * settings.
     */
    protected boolean mRefineResult = DEFAULT_REFINE_RESULT;

    /**
     * Size of subsets to be checked during robust estimation.
     */
    protected int mPreliminarySubsetSize = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Inner non-robust calibrator.
     */
    private final KnownBiasEasyGyroscopeCalibrator mInnerCalibrator = new KnownBiasEasyGyroscopeCalibrator();

    /**
     * Contains normalized start gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D mStartPoint = new InhomogeneousPoint3D();

    /**
     * Contains estimated normalized end gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D mEndPoint = new InhomogeneousPoint3D();

    /**
     * Contains expected normalized end gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D mExpectedEndPoint = new InhomogeneousPoint3D();

    /**
     * Contains amount of rotation for a given sequence and preliminary
     * solution.
     * This is reused when computing error residuals.
     */
    private final Quaternion mQ = new Quaternion();

    /**
     * Array containing measured specific force coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] mMeasuredSpecificForce = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing fixed specific force coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] mFixedSpecificForce = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing measured angular rate coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] mMeasuredAngularRate = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing fixed angular rate coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] mFixedAngularRate = new double[BodyKinematics.COMPONENTS];

    /**
     * An acceleration fixer.
     * This is reused when computing error residuals.
     */
    private final AccelerationFixer mAccelerationFixer = new AccelerationFixer();

    /**
     * An angular rate fixer.
     * This is reused when computing error residuals.
     */
    private final AngularRateFixer mAngularRateFixer = new AngularRateFixer();

    /**
     * Constructor.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator() {
        try {
            mInitialGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg) {
        this();
        mSequences = sequences;
        try {
            setBias(bias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must be 3x1 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @param listener  listener to handle events raised by this
     *                  calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final double[] bias,
            final Matrix initialMg, final Matrix initialGg) {
        this();
        mSequences = sequences;
        try {
            setBias(bias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param sequences collection of sequences containing timestamped body
     *                  kinematics measurements.
     * @param bias      gyroscope known bias. This must have length 3 and is
     *                  expressed in radians per second (rad/s).
     * @param initialMg initial gyroscope scale factors and cross coupling
     *                  errors matrix. Must be 3x3.
     * @param initialGg initial gyroscope G-dependent cross biases
     *                  introduced on the gyroscope by the specific forces
     *                  sensed by the accelerometer. Must be 3x3.
     * @param listener  listener to handle events raised by this
     *                  calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final double[] bias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must have length 3 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final double[] bias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg);
        try {
            setAccelerometerBias(accelerometerBias);
            setAccelerometerMa(accelerometerMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must have length 3 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final double[] bias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must be 3x1 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg);
        try {
            setAccelerometerBias(accelerometerBias);
            setAccelerometerMa(accelerometerMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param bias              gyroscope known bias. This must be 3x1 and is
     *                          expressed in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg) {
        this(sequences, bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] bias,
            final Matrix initialMg, final Matrix initialGg) {
        this(sequences, bias, initialMg, initialGg);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] bias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg, initialGg);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] bias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must have length 3 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] bias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(sequences, bias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        mCommonAxisUsed = commonAxisUsed;
        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param bias                          gyroscope known bias. This must be 3x1 and is
     *                                      expressed in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix bias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        this(sequences, commonAxisUsed, estimateGDependentCrossBiases, bias, initialMg, initialGg, accelerometerBias,
                accelerometerMa);
        mListener = listener;
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known x-coordinate of accelerometer bias.
     */
    @Override
    public double getAccelerometerBiasX() {
        return mAccelerometerBiasX;
    }

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX known x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasX(final double accelerometerBiasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasX = accelerometerBiasX;
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known y-coordinate of accelerometer bias.
     */
    @Override
    public double getAccelerometerBiasY() {
        return mAccelerometerBiasY;
    }

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasY known y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasY(final double accelerometerBiasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasY = accelerometerBiasY;
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known z-coordinate of accelerometer bias.
     */
    @Override
    public double getAccelerometerBiasZ() {
        return mAccelerometerBiasZ;
    }

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasZ known z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasZ(final double accelerometerBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasZ = accelerometerBiasZ;
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known x-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getAccelerometerBiasXAsAcceleration() {
        return new Acceleration(mAccelerometerBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getAccelerometerBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known x-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasX(final Acceleration accelerometerBiasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasX = convertAcceleration(accelerometerBiasX);
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known y-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getAccelerometerBiasYAsAcceleration() {
        return new Acceleration(mAccelerometerBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getAccelerometerBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known y-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasY(final Acceleration accelerometerBiasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasY = convertAcceleration(accelerometerBiasY);
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known z-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getAccelerometerBiasZAsAcceleration() {
        return new Acceleration(mAccelerometerBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getAccelerometerBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerometerBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known z-coordinate of accelerometer bias to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBiasZ(final Acceleration accelerometerBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBias(
            final double accelerometerBiasX, final double accelerometerBiasY, final double accelerometerBiasZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerBiasX = accelerometerBiasX;
        mAccelerometerBiasY = accelerometerBiasY;
        mAccelerometerBiasZ = accelerometerBiasZ;
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerBiasX x-coordinate of accelerometer bias.
     * @param accelerometerBiasY y-coordinate of accelerometer bias.
     * @param accelerometerBiasZ z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerBias(
            final Acceleration accelerometerBiasX, final Acceleration accelerometerBiasY,
            final Acceleration accelerometerBiasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mAccelerometerBiasX = convertAcceleration(accelerometerBiasX);
        mAccelerometerBiasY = convertAcceleration(accelerometerBiasY);
        mAccelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    @Override
    public double[] getAccelerometerBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getAccelerometerBias(result);
        return result;
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void getAccelerometerBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result[0] = mAccelerometerBiasX;
        result[1] = mAccelerometerBiasY;
        result[2] = mAccelerometerBiasZ;
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void setAccelerometerBias(final double[] accelerometerBias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (accelerometerBias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mAccelerometerBiasX = accelerometerBias[0];
        mAccelerometerBiasY = accelerometerBias[1];
        mAccelerometerBiasZ = accelerometerBias[2];
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return known accelerometer bias.
     */
    @Override
    public Matrix getAccelerometerBiasAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, 1);
            getAccelerometerBiasAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getAccelerometerBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mAccelerometerBiasX);
        result.setElementAtIndex(1, mAccelerometerBiasY);
        result.setElementAtIndex(2, mAccelerometerBiasZ);
    }

    /**
     * Sets known accelerometer bias to be used to fix measured specific
     * force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param accelerometerBias known accelerometer bias. Must be 3x1.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setAccelerometerBias(final Matrix accelerometerBias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (accelerometerBias.getRows() != BodyKinematics.COMPONENTS || accelerometerBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mAccelerometerBiasX = accelerometerBias.getElementAtIndex(0);
        mAccelerometerBiasY = accelerometerBias.getElementAtIndex(1);
        mAccelerometerBiasZ = accelerometerBias.getElementAtIndex(2);
    }

    /**
     * Gets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer x scaling factor.
     */
    @Override
    public double getAccelerometerSx() {
        return mAccelerometerSx;
    }

    /**
     * Sets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerSx(final double accelerometerSx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSx = accelerometerSx;
    }

    /**
     * Gets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer y scaling factor.
     */
    @Override
    public double getAccelerometerSy() {
        return mAccelerometerSy;
    }

    /**
     * Sets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSy known accelerometer y scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerSy(final double accelerometerSy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSy = accelerometerSy;
    }

    /**
     * Gets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer z scaling factor.
     */
    @Override
    public double getAccelerometerSz() {
        return mAccelerometerSz;
    }

    /**
     * Sets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerSz(final double accelerometerSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSz = accelerometerSz;
    }

    /**
     * Gets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-y cross coupling error.
     */
    @Override
    public double getAccelerometerMxy() {
        return mAccelerometerMxy;
    }

    /**
     * Sets known accelerometer x-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMxy(final double accelerometerMxy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxy = accelerometerMxy;
    }

    /**
     * Gets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer x-z cross coupling error.
     */
    @Override
    public double getAccelerometerMxz() {
        return mAccelerometerMxz;
    }

    /**
     * Sets known accelerometer x-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxz known accelerometer x-z cross coupling error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMxz(final double accelerometerMxz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxz = accelerometerMxz;
    }

    /**
     * Gets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-x cross coupling error.
     */
    @Override
    public double getAccelerometerMyx() {
        return mAccelerometerMyx;
    }

    /**
     * Sets known accelerometer y-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMyx(final double accelerometerMyx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMyx = accelerometerMyx;
    }

    /**
     * Gets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer y-z cross coupling error.
     */
    @Override
    public double getAccelerometerMyz() {
        return mAccelerometerMyz;
    }

    /**
     * Sets known accelerometer y-z cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMyz(final double accelerometerMyz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMyz = accelerometerMyz;
    }

    /**
     * Gets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-x cross coupling error.
     */
    @Override
    public double getAccelerometerMzx() {
        return mAccelerometerMzx;
    }

    /**
     * Sets known accelerometer z-x cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMzx(final double accelerometerMzx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMzx = accelerometerMzx;
    }

    /**
     * Gets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @return known accelerometer z-y cross coupling error.
     */
    @Override
    public double getAccelerometerMzy() {
        return mAccelerometerMzy;
    }

    /**
     * Sets known accelerometer z-y cross coupling error to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerMzy(final double accelerometerMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMzy = accelerometerMzy;
    }

    /**
     * Sets known accelerometer scaling factors to be used to fix measured
     * specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerSx known accelerometer x scaling factor.
     * @param accelerometerSy known accelerometer y scaling factor.
     * @param accelerometerSz known accelerometer z scaling factor.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerScalingFactors(
            final double accelerometerSx, final double accelerometerSy, final double accelerometerSz)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerSx = accelerometerSx;
        mAccelerometerSy = accelerometerSy;
        mAccelerometerSz = accelerometerSz;
    }

    /**
     * Sets known accelerometer cross coupling errors to be used to fix
     * measured specific force and find cross biases introduced by the
     * accelerometer.
     *
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerCrossCouplingErrors(
            final double accelerometerMxy, final double accelerometerMxz,
            final double accelerometerMyx, final double accelerometerMyz,
            final double accelerometerMzx, final double accelerometerMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mAccelerometerMxy = accelerometerMxy;
        mAccelerometerMxz = accelerometerMxz;
        mAccelerometerMyx = accelerometerMyx;
        mAccelerometerMyz = accelerometerMyz;
        mAccelerometerMzx = accelerometerMzx;
        mAccelerometerMzy = accelerometerMzy;
    }

    /**
     * Sets known accelerometer scaling factors and cross coupling errors
     * to be used to fix measured specific force and find cross biases
     * introduced by the accelerometer.
     *
     * @param accelerometerSx  known accelerometer x scaling factor.
     * @param accelerometerSy  known accelerometer y scaling factor.
     * @param accelerometerSz  known accelerometer z scaling factor.
     * @param accelerometerMxy known accelerometer x-y cross coupling
     *                         error.
     * @param accelerometerMxz known accelerometer x-z cross coupling
     *                         error.
     * @param accelerometerMyx known accelerometer y-x cross coupling
     *                         error.
     * @param accelerometerMyz known accelerometer y-z cross coupling
     *                         error.
     * @param accelerometerMzx known accelerometer z-x cross coupling
     *                         error.
     * @param accelerometerMzy known accelerometer z-y cross coupling
     *                         error.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setAccelerometerScalingFactorsAndCrossCouplingErrors(
            final double accelerometerSx, final double accelerometerSy,
            final double accelerometerSz, final double accelerometerMxy,
            final double accelerometerMxz, final double accelerometerMyx,
            final double accelerometerMyz, final double accelerometerMzx,
            final double accelerometerMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setAccelerometerScalingFactors(accelerometerSx, accelerometerSy, accelerometerSz);
        setAccelerometerCrossCouplingErrors(accelerometerMxy, accelerometerMxz, accelerometerMyx,
                accelerometerMyz, accelerometerMzx, accelerometerMzy);
    }

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @return known accelerometer scale factors and cross coupling
     * errors matrix.
     */
    @Override
    public Matrix getAccelerometerMa() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            getAccelerometerMa(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void getAccelerometerMa(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mAccelerometerSx);
        result.setElementAtIndex(1, mAccelerometerMyx);
        result.setElementAtIndex(2, mAccelerometerMzx);

        result.setElementAtIndex(3, mAccelerometerMxy);
        result.setElementAtIndex(4, mAccelerometerSy);
        result.setElementAtIndex(5, mAccelerometerMzy);

        result.setElementAtIndex(6, mAccelerometerMxz);
        result.setElementAtIndex(7, mAccelerometerMyz);
        result.setElementAtIndex(8, mAccelerometerSz);
    }

    /**
     * Sets known accelerometer scale factors and cross coupling
     * errors matrix.
     *
     * @param accelerometerMa known accelerometer scale factors and
     *                        cross coupling errors matrix. Must be 3x3.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void setAccelerometerMa(final Matrix accelerometerMa) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (accelerometerMa.getRows() != BodyKinematics.COMPONENTS
                || accelerometerMa.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mAccelerometerSx = accelerometerMa.getElementAtIndex(0);
        mAccelerometerMyx = accelerometerMa.getElementAtIndex(1);
        mAccelerometerMzx = accelerometerMa.getElementAtIndex(2);

        mAccelerometerMxy = accelerometerMa.getElementAtIndex(3);
        mAccelerometerSy = accelerometerMa.getElementAtIndex(4);
        mAccelerometerMzy = accelerometerMa.getElementAtIndex(5);

        mAccelerometerMxz = accelerometerMa.getElementAtIndex(6);
        mAccelerometerMyz = accelerometerMa.getElementAtIndex(7);
        mAccelerometerSz = accelerometerMa.getElementAtIndex(8);
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasX() {
        return mBiasX;
    }

    /**
     * Sets x-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasY() {
        return mBiasY;
    }

    /**
     * Sets y-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasY y-coordinate of gyroscope known  bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = biasY;
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return z-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasZ() {
        return mBiasZ;
    }

    /**
     * Sets z-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @param biasZ z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = biasZ;
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(mBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(mBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets x-coordinate of gyroscope known bias.
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasX(final AngularSpeed biasX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAngularSpeed(biasX);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(mBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(mBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets y-coordinate of gyroscope known bias.
     *
     * @param biasY y-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasY(final AngularSpeed biasY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasY = convertAngularSpeed(biasY);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @return initial z-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(mBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(mBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets z-coordinate of gyroscope known bias.
     *
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasZ(final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Sets known bias coordinates of gyroscope expressed in
     * radians per second (rad/s).
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @param biasY y-coordinate of gyroscope known bias.
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final double biasX, final double biasY, final double biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = biasX;
        mBiasY = biasY;
        mBiasZ = biasZ;
    }

    /**
     * Sets known bias coordinates of gyroscope.
     *
     * @param biasX x-coordinate of gyroscope known bias.
     * @param biasY y-coordinate of gyroscope known bias.
     * @param biasZ z-coordinate of gyroscope known bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mBiasX = convertAngularSpeed(biasX);
        mBiasY = convertAngularSpeed(biasY);
        mBiasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @return known gyroscope bias.
     */
    public AngularSpeedTriad getBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, mBiasX, mBiasY, mBiasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(mBiasX, mBiasY, mBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known gyroscope bias.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final AngularSpeedTriad bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mBiasX = convertAngularSpeed(bias.getValueX(), bias.getUnit());
        mBiasY = convertAngularSpeed(bias.getValueY(), bias.getUnit());
        mBiasZ = convertAngularSpeed(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets initial x scaling factor of gyroscope.
     *
     * @return initial x scaling factor of gyroscope.
     */
    @Override
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSx(final double initialSx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
    }

    /**
     * Gets initial y scaling factor of gyroscope.
     *
     * @return initial y scaling factor of gyroscope.
     */
    @Override
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor of gyroscope.
     *
     * @param initialSy initial y scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSy(final double initialSy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSy = initialSy;
    }

    /**
     * Gets initial z scaling factor of gyroscope.
     *
     * @return initial z scaling factor of gyroscope.
     */
    @Override
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor of gyroscope.
     *
     * @param initialSz initial z scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialSz(final double initialSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSz = initialSz;
    }

    /**
     * Gets initial x-y cross coupling error of gyroscope.
     *
     * @return initial x-y cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMxy(final double initialMxy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxy = initialMxy;
    }

    /**
     * Gets initial x-z cross coupling error of gyroscope.
     *
     * @return initial x-z cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error of gyroscope.
     *
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMxz(final double initialMxz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxz = initialMxz;
    }

    /**
     * Gets initial y-x cross coupling error of gyroscope.
     *
     * @return initial y-x cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error of gyroscope.
     *
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMyx(final double initialMyx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyx = initialMyx;
    }

    /**
     * Gets initial y-z cross coupling error of gyroscope.
     *
     * @return initial y-z cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error of gyroscope.
     *
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMyz(final double initialMyz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMyz = initialMyz;
    }

    /**
     * Gets initial z-x cross coupling error of gyroscope.
     *
     * @return initial z-x cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error of gyroscope.
     *
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMzx(final double initialMzx) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzx = initialMzx;
    }

    /**
     * Gets initial z-y cross coupling error of gyroscope.
     *
     * @return initial z-y cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error of gyroscope.
     *
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialMzy(final double initialMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
     * @param initialSy initial y scaling factor of gyroscope.
     * @param initialSz initial z scaling factor of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialScalingFactors(
            final double initialSx, final double initialSy, final double initialSz) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
        mInitialSy = initialSy;
        mInitialSz = initialSz;
    }

    /**
     * Sets initial cross coupling errors of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialCrossCouplingErrors(
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialMxy = initialMxy;
        mInitialMxz = initialMxz;
        mInitialMyx = initialMyx;
        mInitialMyz = initialMyz;
        mInitialMzx = initialMzx;
        mInitialMzy = initialMzy;
    }

    /**
     * Sets initial scaling factors and cross coupling errors of
     * gyroscope.
     *
     * @param initialSx  initial x scaling factor of gyroscope.
     * @param initialSy  initial y scaling factor of gyroscope.
     * @param initialSz  initial z scaling factor of gyroscope.
     * @param initialMxy initial x-y cross coupling error of gyroscope.
     * @param initialMxz initial x-z cross coupling error of gyroscope.
     * @param initialMyx initial y-x cross coupling error of gyroscope.
     * @param initialMyz initial y-z cross coupling error of gyroscope.
     * @param initialMzx initial z-x cross coupling error of gyroscope.
     * @param initialMzy initial z-y cross coupling error of gyroscope.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialScalingFactorsAndCrossCouplingErrors(
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        setInitialScalingFactors(initialSx, initialSy, initialSz);
        setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Gets gyroscope known bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of gyroscope known bias.
     */
    @Override
    public double[] getBias() {
        final double[] result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets gyroscope known bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void getBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mBiasX;
        result[1] = mBiasY;
        result[2] = mBiasZ;
    }

    /**
     * Sets gyroscope known bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setBias(final double[] bias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mBiasX = bias[0];
        mBiasY = bias[1];
        mBiasZ = bias[2];
    }

    /**
     * Gets gyroscope known bias as a column matrix.
     *
     * @return initial gyroscope bias to be used to find a solution as a
     * column matrix.
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
     * Gets gyroscope known bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mBiasX);
        result.setElementAtIndex(1, mBiasY);
        result.setElementAtIndex(2, mBiasZ);
    }

    /**
     * Sets gyroscope known bias as a column matrix.
     *
     * @param initialBias gyroscope known bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setBias(final Matrix initialBias) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialBias.getRows() != BodyKinematics.COMPONENTS || initialBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mBiasX = initialBias.getElementAtIndex(0);
        mBiasY = initialBias.getElementAtIndex(1);
        mBiasZ = initialBias.getElementAtIndex(2);
    }

    /**
     * Gets initial gyroscope scale factors and cross coupling errors
     * matrix.
     *
     * @return initial gyroscope scale factors and cross coupling errors
     * matrix.
     */
    @Override
    public Matrix getInitialMg() {
        Matrix result;
        try {
            result = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            getInitialMg(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial gyroscope scale factors and cross coupling errors
     * matrix.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void getInitialMg(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mInitialSx);
        result.setElementAtIndex(1, mInitialMyx);
        result.setElementAtIndex(2, mInitialMzx);

        result.setElementAtIndex(3, mInitialMxy);
        result.setElementAtIndex(4, mInitialSy);
        result.setElementAtIndex(5, mInitialMzy);

        result.setElementAtIndex(6, mInitialMxz);
        result.setElementAtIndex(7, mInitialMyz);
        result.setElementAtIndex(8, mInitialSz);
    }

    /**
     * Sets initial gyroscope scale factors and cross coupling errors matrix.
     *
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setInitialMg(final Matrix initialMg) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialMg.getRows() != BodyKinematics.COMPONENTS || initialMg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mInitialSx = initialMg.getElementAtIndex(0);
        mInitialMyx = initialMg.getElementAtIndex(1);
        mInitialMzx = initialMg.getElementAtIndex(2);

        mInitialMxy = initialMg.getElementAtIndex(3);
        mInitialSy = initialMg.getElementAtIndex(4);
        mInitialMzy = initialMg.getElementAtIndex(5);

        mInitialMxz = initialMg.getElementAtIndex(6);
        mInitialMyz = initialMg.getElementAtIndex(7);
        mInitialSz = initialMg.getElementAtIndex(8);
    }

    /**
     * Gets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing initial g-dependent cross biases.
     */
    @Override
    public Matrix getInitialGg() {
        return new Matrix(mInitialGg);
    }

    /**
     * Gets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @param result instance where data will be stored.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void getInitialGg(final Matrix result) {

        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        result.copyFrom(mInitialGg);
    }

    /**
     * Sets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @param initialGg g-dependent cross biases.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    @Override
    public void setInitialGg(final Matrix initialGg) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialGg.getRows() != BodyKinematics.COMPONENTS || initialGg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialGg.copyTo(mInitialGg);
    }

    /**
     * Gets collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     *
     * @return collection of sequences of timestamped body kinematics
     * measurements.
     */
    @Override
    public List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> getSequences() {
        return mSequences;
    }

    /**
     * Sets collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     *
     * @param sequences collection of sequences of timestamped body
     *                  kinematics measurements.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setSequences(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mSequences = sequences;
    }

    /**
     * Indicates the type of measurement or sequence used by this calibrator.
     *
     * @return type of measurement or sequence used by this calibrator.
     */
    @Override
    public GyroscopeCalibratorMeasurementOrSequenceType getMeasurementOrSequenceType() {
        return GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE;
    }

    /**
     * Indicates whether this calibrator requires ordered measurements or sequences
     * in a list or not.
     *
     * @return true if measurements or sequences must be ordered, false otherwise.
     */
    @Override
    public boolean isOrderedMeasurementsOrSequencesRequired() {
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
        return mCommonAxisUsed;
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
        if (mRunning) {
            throw new LockedException();
        }

        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Indicates whether G-dependent cross biases are being estimated
     * or not.
     * When enabled, this adds 9 variables from Gg matrix.
     *
     * @return true if G-dependent cross biases will be estimated,
     * false otherwise.
     */
    public boolean isGDependentCrossBiasesEstimated() {
        return mEstimateGDependentCrossBiases;
    }

    /**
     * Specifies whether G-dependent cross biases are being estimated
     * or not.
     * When enabled, this adds 9 variables from Gg matrix.
     *
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @throws LockedException if calibrator is currently running.
     */
    public void setGDependentCrossBiasesEstimated(final boolean estimateGDependentCrossBiases) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mEstimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RobustKnownBiasEasyGyroscopeCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownBiasEasyGyroscopeCalibratorListener listener) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum number of required sequences.
     *
     * @return minimum number of required sequences.
     */
    @Override
    public int getMinimumRequiredMeasurementsOrSequences() {
        if (mCommonAxisUsed) {
            if (mEstimateGDependentCrossBiases) {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            } else {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            }
        } else {
            if (mEstimateGDependentCrossBiases) {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES;
            } else {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL;
            }
        }
    }

    /**
     * Indicates whether calibrator is ready to start.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mSequences != null && mSequences.size() >= getMinimumRequiredMeasurementsOrSequences();
    }

    /**
     * Indicates whether calibrator is currently running or not.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Returns amount of progress variation before notifying a progress change during
     * calibration.
     *
     * @return amount of progress variation before notifying a progress change during
     * calibration.
     */
    public float getProgressDelta() {
        return mProgressDelta;
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
        if (mRunning) {
            throw new LockedException();
        }
        if (progressDelta < MIN_PROGRESS_DELTA || progressDelta > MAX_PROGRESS_DELTA) {
            throw new IllegalArgumentException();
        }
        mProgressDelta = progressDelta;
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
        return mConfidence;
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
        if (mRunning) {
            throw new LockedException();
        }
        if (confidence < MIN_CONFIDENCE || confidence > MAX_CONFIDENCE) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
    }

    /**
     * Returns maximum allowed number of iterations. If maximum allowed number of
     * iterations is achieved without converging to a result when calling calibrate(),
     * a RobustEstimatorException will be raised.
     *
     * @return maximum allowed number of iterations.
     */
    public int getMaxIterations() {
        return mMaxIterations;
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
        if (mRunning) {
            throw new LockedException();
        }
        if (maxIterations < MIN_ITERATIONS) {
            throw new IllegalArgumentException();
        }
        mMaxIterations = maxIterations;
    }

    /**
     * Gets data related to inliers found after estimation.
     *
     * @return data related to inliers found after estimation.
     */
    public InliersData getInliersData() {
        return mInliersData;
    }

    /**
     * Indicates whether result must be refined using a non-linear solver over found inliers.
     *
     * @return true to refine result, false to simply use result found by robust estimator
     * without further refining.
     */
    public boolean isResultRefined() {
        return mRefineResult;
    }

    /**
     * Specifies whether result must be refined using a non-linear solver over found inliers.
     *
     * @param refineResult true to refine result, false to simply use result found by robust
     *                     estimator without further refining.
     * @throws LockedException if calibrator is currently running.
     */
    public void setResultRefined(final boolean refineResult) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mRefineResult = refineResult;
    }

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     *
     * @return true if covariance must be kept after refining result, false otherwise.
     */
    public boolean isCovarianceKept() {
        return mKeepCovariance;
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
        if (mRunning) {
            throw new LockedException();
        }
        mKeepCovariance = keepCovariance;
    }

    /**
     * Returns quality scores corresponding to each sequence.
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
     * Sets quality scores corresponding to each sequence.
     * The larger the score value the better the quality of the sample.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     *
     * @param qualityScores quality scores corresponding to each sample.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
    }

    /**
     * Gets estimated gyroscope scale factors and cross coupling errors.
     * This is the product of matrix Tg containing cross coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated gyroscope scale factors and cross coupling errors, or null
     * if not available.
     */
    @Override
    public Matrix getEstimatedMg() {
        return mEstimatedMg;
    }

    /**
     * Gets estimated gyroscope x-axis scale factor.
     *
     * @return estimated gyroscope x-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-axis scale factor.
     *
     * @return estimated gyroscope y-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated gyroscope z-axis scale factor.
     *
     * @return estimated gyroscope z-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated gyroscope x-y cross-coupling error.
     *
     * @return estimated gyroscope x-y cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated gyroscope x-z cross-coupling error.
     *
     * @return estimated gyroscope x-z cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated gyroscope y-x cross-coupling error.
     *
     * @return estimated gyroscope y-x cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-z cross-coupling error.
     *
     * @return estimated gyroscope y-z cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated gyroscope z-x cross-coupling error.
     *
     * @return estimated gyroscope z-x cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated gyroscope z-y cross-coupling error.
     *
     * @return estimated gyroscope z-y cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMg != null ? mEstimatedMg.getElementAt(2, 1) : null;
    }

    /**
     * Gets estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     *
     * @return estimated G-dependent cross biases.
     */
    @Override
    public Matrix getEstimatedGg() {
        return mEstimatedGg;
    }

    /**
     * Gets estimated covariance matrix for estimated parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
     *
     * @return estimated covariance matrix for estimated parameters.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets estimated chi square value.
     *
     * @return estimated chi square value.
     */
    @Override
    public double getEstimatedChiSq() {
        return mEstimatedChiSq;
    }

    /**
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    @Override
    public double getEstimatedMse() {
        return mEstimatedMse;
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurementsOrSequences()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #getMinimumRequiredMeasurementsOrSequences}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link #getMinimumRequiredMeasurementsOrSequences}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinimumRequiredMeasurementsOrSequences()) {
            throw new IllegalArgumentException();
        }

        mPreliminarySubsetSize = preliminarySubsetSize;
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    public abstract RobustEstimatorMethod getMethod();

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator();
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator();
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator();
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator();
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator();
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param method      robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @param method      robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param method      robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @param method      robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        switch (method) {
            case RANSAC:
                return new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                        sequences, initialBias, initialMg, initialGg, listener);
            case LMEDS:
                return new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                        sequences, initialBias, initialMg, initialGg, listener);
            case MSAC:
                return new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                        sequences, initialBias, initialMg, initialGg, listener);
            case PROSAC:
                return new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                        sequences, initialBias, initialMg, initialGg, listener);
            case PROMEDS:
            default:
                return new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                        sequences, initialBias, initialMg, initialGg, listener);
        }
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, initialBias, initialMg,
                    initialGg, accelerometerBias, accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                    accelerometerBias, accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                    accelerometerBias, accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                    accelerometerBias, accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                    accelerometerBias, accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                    accelerometerBias, accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator();
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator();
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator();
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param initialBias   initial gyroscope bias to be used to find a solution.
     *                      This must be 3x1 and is expressed in radians per
     *                      second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param initialBias   initial gyroscope bias to be used to find a solution.
     *                      This must be 3x1 and is expressed in radians per
     *                      second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param listener      listener to handle events raised by this
     *                      calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param initialBias   initial gyroscope bias to be used to find a
     *                      solution. This must have length 3 and is expressed
     *                      in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      sequence. The larger the score value the better
     *                      the quality of the sequence.
     * @param sequences     collection of sequences containing timestamped body
     *                      kinematics measurements.
     * @param initialBias   initial gyroscope bias to be used to find a
     *                      solution. This must have length 3 and is expressed
     *                      in radians per second (rad/s).
     * @param initialMg     initial gyroscope scale factors and cross coupling
     *                      errors matrix. Must be 3x3.
     * @param initialGg     initial gyroscope G-dependent cross biases
     *                      introduced on the gyroscope by the specific forces
     *                      sensed by the accelerometer. Must be 3x3.
     * @param listener      listener to handle events raised by this
     *                      calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                    listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                    listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores     quality scores corresponding to each provided
     *                          sequence. The larger the score value the better
     *                          the quality of the sequence.
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @param method            robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(
                    sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                    listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(
                    qualityScores, sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                    listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores                 quality scores corresponding to each provided
     *                                      sequence. The larger the score value the better
     *                                      the quality of the sequence.
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @param method                        robust estimator method.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if provided
     *                                  quality scores length is smaller
     *                                  than 10.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final double[] qualityScores,
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case LMEDS -> new LMedSRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case MSAC -> new MSACRobustKnownBiasEasyGyroscopeCalibrator(sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            case PROSAC -> new PROSACRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
            default -> new PROMedSRobustKnownBiasEasyGyroscopeCalibrator(qualityScores, sequences, commonAxisUsed,
                    estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias,
                    accelerometerMa, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg) {
        return create(sequences, initialBias, initialMg, initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a solution.
     *                    This must be 3x1 and is expressed in radians per
     *                    second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, initialBias, initialMg, initialGg, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg) {
        return create(sequences, initialBias, initialMg, initialGg, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences   collection of sequences containing timestamped body
     *                    kinematics measurements.
     * @param initialBias initial gyroscope bias to be used to find a
     *                    solution. This must have length 3 and is expressed
     *                    in radians per second (rad/s).
     * @param initialMg   initial gyroscope scale factors and cross coupling
     *                    errors matrix. Must be 3x3.
     * @param initialGg   initial gyroscope G-dependent cross biases
     *                    introduced on the gyroscope by the specific forces
     *                    sensed by the accelerometer. Must be 3x3.
     * @param listener    listener to handle events raised by this
     *                    calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, initialBias, initialMg, initialGg, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa) {
        return create(sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must have length 3 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must
     *                          have length 3 and is expressed in
     *                          meters per squared second
     *                          (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final double[] initialBias, final Matrix initialMg, final Matrix initialGg,
            final double[] accelerometerBias, final Matrix accelerometerMa,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences         collection of sequences containing timestamped body
     *                          kinematics measurements.
     * @param initialBias       initial gyroscope bias to be used to find a
     *                          solution. This must be 3x1 and is expressed
     *                          in radians per second (rad/s).
     * @param initialMg         initial gyroscope scale factors and cross coupling
     *                          errors matrix. Must be 3x3.
     * @param initialGg         initial gyroscope G-dependent cross biases
     *                          introduced on the gyroscope by the specific forces
     *                          sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias known accelerometer bias. This must be 3x1
     *                          and is expressed in meters per squared
     *                          second (m/s^2).
     * @param accelerometerMa   known accelerometer scale factors and
     *                          cross coupling matrix. Must be 3x3.
     * @param listener          listener to handle events raised by this
     *                          calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final Matrix initialBias, final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg,
            final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param sequences                     collection of sequences containing timestamped body
     *                                      kinematics measurements.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used to find a
     *                                      solution. This must be 3x1 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors and cross coupling
     *                                      errors matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent cross biases
     *                                      introduced on the gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised by this
     *                                      calibrator.
     * @return a robust gyroscope calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size.
     */
    public static RobustKnownBiasEasyGyroscopeCalibrator create(
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences,
            final boolean commonAxisUsed, final boolean estimateGDependentCrossBiases, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final RobustKnownBiasEasyGyroscopeCalibratorListener listener) {
        return create(sequences, commonAxisUsed, estimateGDependentCrossBiases, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Configures acceleration fixer
     *
     * @throws AlgebraException if provided accelerometer parameters
     *                          are numerically unstable.
     */
    protected void setupAccelerationFixer() throws AlgebraException {
        mAccelerationFixer.setBias(getAccelerometerBias());
        mAccelerationFixer.setCrossCouplingErrors(getAccelerometerMa());
    }

    /**
     * Computes error of a preliminary result respect a given sequence.
     *
     * @param sequence          a sequence.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence,
            final PreliminaryResult preliminaryResult) {

        try {
            mAngularRateFixer.setBias(mBiasX, mBiasY, mBiasZ);
            mAngularRateFixer.setCrossCouplingErrors(preliminaryResult.mEstimatedMg);
            mAngularRateFixer.setGDependantCrossBias(preliminaryResult.mEstimatedGg);

            // copy measured sequence as it will be used to fix kinematics values
            // using preliminary gyroscope parameters
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> fixedSequence =
                    new BodyKinematicsSequence<>(sequence);

            // fix body kinematic measurements of provided sequence
            final int numItems = sequence.getItemsCount();
            final List<StandardDeviationTimedBodyKinematics> measuredItems = sequence.getSortedItems();
            final List<StandardDeviationTimedBodyKinematics> fixedItems = fixedSequence.getSortedItems();
            for (int j = 0; j < numItems; j++) {
                final StandardDeviationTimedBodyKinematics measuredItem = measuredItems.get(j);
                final StandardDeviationTimedBodyKinematics fixedItem = fixedItems.get(j);

                final BodyKinematics measuredKinematics = measuredItem.getKinematics();
                final BodyKinematics fixedKinematics = fixedItem.getKinematics();
                fixKinematics(measuredKinematics, fixedKinematics);
            }

            // integrate fixed sequence to obtain attitude change
            QuaternionIntegrator.integrateGyroSequence(fixedSequence, QuaternionStepIntegratorType.RUNGE_KUTTA, mQ);

            // fix before coordinates
            mMeasuredSpecificForce[0] = sequence.getBeforeMeanFx();
            mMeasuredSpecificForce[1] = sequence.getBeforeMeanFy();
            mMeasuredSpecificForce[2] = sequence.getBeforeMeanFz();
            mAccelerationFixer.fix(mMeasuredSpecificForce, mFixedSpecificForce);

            // normalize coordinates
            ArrayUtils.normalize(mFixedSpecificForce);

            // compute estimated normalized end coordinates
            mStartPoint.setCoordinates(mFixedSpecificForce);
            mQ.inverse();
            mQ.rotate(mStartPoint, mEndPoint);

            // fix after coordinates
            mMeasuredSpecificForce[0] = sequence.getAfterMeanFx();
            mMeasuredSpecificForce[1] = sequence.getAfterMeanFy();
            mMeasuredSpecificForce[2] = sequence.getAfterMeanFz();
            mAccelerationFixer.fix(mMeasuredSpecificForce, mFixedSpecificForce);

            // normalize coordinates
            ArrayUtils.normalize(mFixedSpecificForce);

            mExpectedEndPoint.setCoordinates(mFixedSpecificForce);

            // compare estimated normalized end coordinates with expected
            // ones
            return mExpectedEndPoint.distanceTo(mEndPoint);

        } catch (final AlgebraException | RotationException e) {
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
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

        for (int samplesIndex : samplesIndices) {
            sequences.add(mSequences.get(samplesIndex));
        }

        try {
            final PreliminaryResult result = new PreliminaryResult();
            result.mEstimatedMg = getInitialMg();
            result.mEstimatedGg = getInitialGg();

            mInnerCalibrator.setGDependentCrossBiasesEstimated(mEstimateGDependentCrossBiases);
            mInnerCalibrator.setBiasCoordinates(mBiasX, mBiasY, mBiasZ);
            mInnerCalibrator.setInitialMg(result.mEstimatedMg);
            mInnerCalibrator.setInitialGg(result.mEstimatedGg);
            mInnerCalibrator.setAccelerometerBias(mAccelerometerBiasX, mAccelerometerBiasY, mAccelerometerBiasZ);
            mInnerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                    mAccelerometerSx, mAccelerometerSy, mAccelerometerSz,
                    mAccelerometerMxy, mAccelerometerMxz, mAccelerometerMyx,
                    mAccelerometerMyz, mAccelerometerMzx, mAccelerometerMzy);
            mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
            mInnerCalibrator.setSequences(sequences);
            mInnerCalibrator.calibrate();

            result.mEstimatedMg = mInnerCalibrator.getEstimatedMg();
            result.mEstimatedGg = mInnerCalibrator.getEstimatedGg();

            if (mKeepCovariance) {
                result.mCovariance = mInnerCalibrator.getEstimatedCovariance();
            } else {
                result.mCovariance = null;
            }

            result.mEstimatedMse = mInnerCalibrator.getEstimatedMse();
            result.mEstimatedChiSq = mInnerCalibrator.getEstimatedChiSq();

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
        if (mRefineResult && mInliersData != null) {
            final BitSet inliers = mInliersData.getInliers();
            final int nSamples = mSequences.size();

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> inlierSequences =
                    new ArrayList<>();
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierSequences.add(mSequences.get(i));
                }
            }

            try {
                mInnerCalibrator.setGDependentCrossBiasesEstimated(mEstimateGDependentCrossBiases);
                mInnerCalibrator.setBiasCoordinates(mBiasX, mBiasY, mBiasZ);
                mInnerCalibrator.setInitialMg(preliminaryResult.mEstimatedMg);
                mInnerCalibrator.setInitialGg(preliminaryResult.mEstimatedGg);
                mInnerCalibrator.setAccelerometerBias(mAccelerometerBiasX, mAccelerometerBiasY, mAccelerometerBiasZ);
                mInnerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                        mAccelerometerSx, mAccelerometerSy, mAccelerometerSz,
                        mAccelerometerMxy, mAccelerometerMxz, mAccelerometerMyx,
                        mAccelerometerMyz, mAccelerometerMzx, mAccelerometerMzy);
                mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mInnerCalibrator.setSequences(inlierSequences);
                mInnerCalibrator.calibrate();

                mEstimatedMg = mInnerCalibrator.getEstimatedMg();
                mEstimatedGg = mInnerCalibrator.getEstimatedGg();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mInnerCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }

                mEstimatedMse = mInnerCalibrator.getEstimatedMse();
                mEstimatedChiSq = mInnerCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = preliminaryResult.mCovariance;
                mEstimatedMg = preliminaryResult.mEstimatedMg;
                mEstimatedGg = preliminaryResult.mEstimatedGg;
                mEstimatedMse = preliminaryResult.mEstimatedMse;
                mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
            }
        } else {
            mEstimatedCovariance = preliminaryResult.mCovariance;
            mEstimatedMg = preliminaryResult.mEstimatedMg;
            mEstimatedGg = preliminaryResult.mEstimatedGg;
            mEstimatedMse = preliminaryResult.mEstimatedMse;
            mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
        }
    }

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param value angular speed value.
     * @param unit  unit of angular speed value.
     * @return converted value.
     */
    private static double convertAngularSpeed(final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second (rad/s).
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(), angularSpeed.getUnit());
    }

    /**
     * Fixes a measured kinematics instance using current
     *
     * @param measuredKinematics a measured kinematics instance.
     * @param result             instance where fixed values will be stored.
     * @throws AlgebraException if accelerometer or gyroscope parameters
     *                          contain numerical instabilities.
     */
    private void fixKinematics(
            final BodyKinematics measuredKinematics, final BodyKinematics result) throws AlgebraException {

        mMeasuredSpecificForce[0] = measuredKinematics.getFx();
        mMeasuredSpecificForce[1] = measuredKinematics.getFy();
        mMeasuredSpecificForce[2] = measuredKinematics.getFz();
        mAccelerationFixer.fix(mMeasuredSpecificForce, mFixedSpecificForce);

        mMeasuredAngularRate[0] = measuredKinematics.getAngularRateX();
        mMeasuredAngularRate[1] = measuredKinematics.getAngularRateY();
        mMeasuredAngularRate[2] = measuredKinematics.getAngularRateZ();
        mAngularRateFixer.fix(mMeasuredAngularRate, mFixedSpecificForce, mFixedAngularRate);

        result.setSpecificForceCoordinates(mFixedSpecificForce[0], mFixedSpecificForce[1], mFixedSpecificForce[2]);
        result.setAngularRateCoordinates(mFixedAngularRate[0], mFixedAngularRate[1], mFixedAngularRate[2]);
    }

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
        /**
         * Estimated gyroscope scale factors and cross coupling errors.
         * This is the product of matrix Tg containing cross coupling errors and Kg
         * containing scaling factors.
         * So that:
         * <pre>
         *     Mg = [sx    mxy  mxz] = Tg*Kg
         *          [myx   sy   myz]
         *          [mzx   mzy  sz ]
         * </pre>
         * Where:
         * <pre>
         *     Kg = [sx 0   0 ]
         *          [0  sy  0 ]
         *          [0  0   sz]
         * </pre>
         * and
         * <pre>
         *     Tg = [1          -alphaXy    alphaXz ]
         *          [alphaYx    1           -alphaYz]
         *          [-alphaZx   alphaZy     1       ]
         * </pre>
         * Hence:
         * <pre>
         *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
         *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
         *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
         * </pre>
         * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
         * are considered to be zero if the gyroscope z-axis is assumed to be the same
         * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
         * becomes upper diagonal:
         * <pre>
         *     Mg = [sx    mxy  mxz]
         *          [0     sy   myz]
         *          [0     0    sz ]
         * </pre>
         * Values of this matrix are unit-less.
         */
        private Matrix mEstimatedMg;

        /**
         * Estimated G-dependent cross biases introduced on the gyroscope by the
         * specific forces sensed by the accelerometer.
         * This instance allows any 3x3 matrix.
         */
        private Matrix mEstimatedGg;

        /**
         * Covariance matrix for estimated result.
         */
        private Matrix mCovariance;

        /**
         * Estimated Mean Square Error.
         */
        private double mEstimatedMse;

        /**
         * Estimated chi square value.
         */
        private double mEstimatedChiSq;
    }
}
