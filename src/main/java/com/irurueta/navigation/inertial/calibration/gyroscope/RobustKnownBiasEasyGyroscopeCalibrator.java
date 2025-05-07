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
    private double accelerometerBiasX;

    /**
     * Known y-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double accelerometerBiasY;

    /**
     * Known z-coordinate of accelerometer bias to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double accelerometerBiasZ;

    /**
     * Known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerSx;

    /**
     * Known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerSy;

    /**
     * Known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerSz;

    /**
     * Known accelerometer x-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMxy;

    /**
     * Know accelerometer x-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMxz;

    /**
     * Known accelerometer y-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMyx;

    /**
     * Known accelerometer y-z cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMyz;

    /**
     * Known accelerometer z-x cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMzx;

    /**
     * Known accelerometer z-y cross coupling error to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     */
    private double accelerometerMzy;

    /**
     * X-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double biasX;

    /**
     * Y-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double biasY;

    /**
     * Z-coordinate of gyroscope known bias expressed in radians per second
     * (rad/s).
     */
    private double biasZ;

    /**
     * Initial gyroscope x scaling factor.
     */
    private double initialSx;

    /**
     * Initial gyroscope y scaling factor.
     */
    private double initialSy;

    /**
     * Initial gyroscope z scaling factor.
     */
    private double initialSz;

    /**
     * Initial gyroscope x-y cross coupling error.
     */
    private double initialMxy;

    /**
     * Initial gyroscope x-z cross coupling error.
     */
    private double initialMxz;

    /**
     * Initial gyroscope y-x cross coupling error.
     */
    private double initialMyx;

    /**
     * Initial gyroscope y-z cross coupling error.
     */
    private double initialMyz;

    /**
     * Initial gyroscope z-x cross coupling error.
     */
    private double initialMzx;

    /**
     * Initial gyroscope z-y cross coupling error.
     */
    private double initialMzy;

    /**
     * Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     */
    private Matrix initialGg;

    /**
     * Contains a collection of sequences of timestamped body kinematics
     * measurements taken at a given position where the device moves freely
     * with different orientations.
     */
    protected List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * This flag indicates whether G-dependent cross biases are being
     * estimated or not.
     * When enabled, this adds 9 variables from Gg matrix.
     */
    private boolean estimateGDependentCrossBiases = DEFAULT_ESTIMATE_G_DEPENDENT_CROSS_BIASES;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownBiasEasyGyroscopeCalibratorListener listener;

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
    private Matrix estimatedMg;

    /**
     * Estimated G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix estimatedGg;

    /**
     * Estimated covariance matrix for estimated parameters.
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
    protected int preliminarySubsetSize = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

    /**
     * Inner non-robust calibrator.
     */
    private final KnownBiasEasyGyroscopeCalibrator innerCalibrator = new KnownBiasEasyGyroscopeCalibrator();

    /**
     * Contains normalized start gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D startPoint = new InhomogeneousPoint3D();

    /**
     * Contains estimated normalized end gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D endPoint = new InhomogeneousPoint3D();

    /**
     * Contains expected normalized end gravity coordinates.
     * This is reused when computing error residuals.
     */
    private final InhomogeneousPoint3D expectedEndPoint = new InhomogeneousPoint3D();

    /**
     * Contains amount of rotation for a given sequence and preliminary
     * solution.
     * This is reused when computing error residuals.
     */
    private final Quaternion q = new Quaternion();

    /**
     * Array containing measured specific force coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] measuredSpecificForce = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing fixed specific force coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] fixedSpecificForce = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing measured angular rate coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] measuredAngularRate = new double[BodyKinematics.COMPONENTS];

    /**
     * Array containing fixed angular rate coordinates.
     * This is reused when computing error residuals.
     */
    private final double[] fixedAngularRate = new double[BodyKinematics.COMPONENTS];

    /**
     * An acceleration fixer.
     * This is reused when computing error residuals.
     */
    private final AccelerationFixer accelerationFixer = new AccelerationFixer();

    /**
     * An angular rate fixer.
     * This is reused when computing error residuals.
     */
    private final AngularRateFixer angularRateFixer = new AngularRateFixer();

    /**
     * Constructor.
     */
    protected RobustKnownBiasEasyGyroscopeCalibrator() {
        try {
            initialGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
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
        this.sequences = sequences;
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
        this.listener = listener;
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
        this.sequences = sequences;
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
        this.listener = listener;
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
        this.listener = listener;
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
        this.listener = listener;
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
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
        this.listener = listener;
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
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
        this.listener = listener;
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
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
        this.listener = listener;
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
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
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
        this.listener = listener;
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
        return accelerometerBiasX;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasX = accelerometerBiasX;
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
        return accelerometerBiasY;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasY = accelerometerBiasY;
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
        return accelerometerBiasZ;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasZ = accelerometerBiasZ;
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
        return new Acceleration(accelerometerBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        result.setValue(accelerometerBiasX);
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasX = convertAcceleration(accelerometerBiasX);
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
        return new Acceleration(accelerometerBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        result.setValue(accelerometerBiasY);
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasY = convertAcceleration(accelerometerBiasY);
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
        return new Acceleration(accelerometerBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
        result.setValue(accelerometerBiasZ);
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
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
        if (running) {
            throw new LockedException();
        }

        this.accelerometerBiasX = accelerometerBiasX;
        this.accelerometerBiasY = accelerometerBiasY;
        this.accelerometerBiasZ = accelerometerBiasZ;
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
        if (running) {
            throw new LockedException();
        }

        this.accelerometerBiasX = convertAcceleration(accelerometerBiasX);
        this.accelerometerBiasY = convertAcceleration(accelerometerBiasY);
        this.accelerometerBiasZ = convertAcceleration(accelerometerBiasZ);
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
        final var result = new double[BodyKinematics.COMPONENTS];
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

        result[0] = accelerometerBiasX;
        result[1] = accelerometerBiasY;
        result[2] = accelerometerBiasZ;
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
        if (running) {
            throw new LockedException();
        }

        if (accelerometerBias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        accelerometerBiasX = accelerometerBias[0];
        accelerometerBiasY = accelerometerBias[1];
        accelerometerBiasZ = accelerometerBias[2];
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
        result.setElementAtIndex(0, accelerometerBiasX);
        result.setElementAtIndex(1, accelerometerBiasY);
        result.setElementAtIndex(2, accelerometerBiasZ);
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
        if (running) {
            throw new LockedException();
        }
        if (accelerometerBias.getRows() != BodyKinematics.COMPONENTS || accelerometerBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        accelerometerBiasX = accelerometerBias.getElementAtIndex(0);
        accelerometerBiasY = accelerometerBias.getElementAtIndex(1);
        accelerometerBiasZ = accelerometerBias.getElementAtIndex(2);
    }

    /**
     * Gets known accelerometer x scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer x scaling factor.
     */
    @Override
    public double getAccelerometerSx() {
        return accelerometerSx;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerSx = accelerometerSx;
    }

    /**
     * Gets known accelerometer y scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer y scaling factor.
     */
    @Override
    public double getAccelerometerSy() {
        return accelerometerSy;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerSy = accelerometerSy;
    }

    /**
     * Gets known accelerometer z scaling factor to be used to fix measured
     * specific force and find cross biases introduced by the accelerometer.
     *
     * @return known accelerometer z scaling factor.
     */
    @Override
    public double getAccelerometerSz() {
        return accelerometerSz;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerSz = accelerometerSz;
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
        return accelerometerMxy;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMxy = accelerometerMxy;
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
        return accelerometerMxz;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMxz = accelerometerMxz;
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
        return accelerometerMyx;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMyx = accelerometerMyx;
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
        return accelerometerMyz;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMyz = accelerometerMyz;
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
        return accelerometerMzx;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMzx = accelerometerMzx;
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
        return accelerometerMzy;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMzy = accelerometerMzy;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerSx = accelerometerSx;
        this.accelerometerSy = accelerometerSy;
        this.accelerometerSz = accelerometerSz;
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
        if (running) {
            throw new LockedException();
        }
        this.accelerometerMxy = accelerometerMxy;
        this.accelerometerMxz = accelerometerMxz;
        this.accelerometerMyx = accelerometerMyx;
        this.accelerometerMyz = accelerometerMyz;
        this.accelerometerMzx = accelerometerMzx;
        this.accelerometerMzy = accelerometerMzy;
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
        if (running) {
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
        result.setElementAtIndex(0, accelerometerSx);
        result.setElementAtIndex(1, accelerometerMyx);
        result.setElementAtIndex(2, accelerometerMzx);

        result.setElementAtIndex(3, accelerometerMxy);
        result.setElementAtIndex(4, accelerometerSy);
        result.setElementAtIndex(5, accelerometerMzy);

        result.setElementAtIndex(6, accelerometerMxz);
        result.setElementAtIndex(7, accelerometerMyz);
        result.setElementAtIndex(8, accelerometerSz);
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
        if (running) {
            throw new LockedException();
        }
        if (accelerometerMa.getRows() != BodyKinematics.COMPONENTS
                || accelerometerMa.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        accelerometerSx = accelerometerMa.getElementAtIndex(0);
        accelerometerMyx = accelerometerMa.getElementAtIndex(1);
        accelerometerMzx = accelerometerMa.getElementAtIndex(2);

        accelerometerMxy = accelerometerMa.getElementAtIndex(3);
        accelerometerSy = accelerometerMa.getElementAtIndex(4);
        accelerometerMzy = accelerometerMa.getElementAtIndex(5);

        accelerometerMxz = accelerometerMa.getElementAtIndex(6);
        accelerometerMyz = accelerometerMa.getElementAtIndex(7);
        accelerometerSz = accelerometerMa.getElementAtIndex(8);
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasX() {
        return biasX;
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
        if (running) {
            throw new LockedException();
        }
        this.biasX = biasX;
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasY() {
        return biasY;
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
        if (running) {
            throw new LockedException();
        }
        this.biasY = biasY;
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     * This is expressed in radians per second (rad/s).
     *
     * @return z-coordinate of gyroscope known bias.
     */
    @Override
    public double getBiasZ() {
        return biasZ;
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
        if (running) {
            throw new LockedException();
        }
        this.biasZ = biasZ;
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @return x-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets x-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(biasX);
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
        if (running) {
            throw new LockedException();
        }
        this.biasX = convertAngularSpeed(biasX);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @return y-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets y-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(biasY);
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
        if (running) {
            throw new LockedException();
        }
        this.biasY = convertAngularSpeed(biasY);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @return initial z-coordinate of gyroscope known bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets z-coordinate of gyroscope known bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(biasZ);
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
        if (running) {
            throw new LockedException();
        }
        this.biasZ = convertAngularSpeed(biasZ);
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
        if (running) {
            throw new LockedException();
        }
        this.biasX = biasX;
        this.biasY = biasY;
        this.biasZ = biasZ;
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
        if (running) {
            throw new LockedException();
        }
        this.biasX = convertAngularSpeed(biasX);
        this.biasY = convertAngularSpeed(biasY);
        this.biasZ = convertAngularSpeed(biasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @return known gyroscope bias.
     */
    public AngularSpeedTriad getBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, biasX, biasY, biasZ);
    }

    /**
     * Gets known gyroscope bias.
     *
     * @param result instance where result will be stored.
     */
    public void getBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(biasX, biasY, biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known gyroscope bias.
     *
     * @param bias gyroscope bias to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setBias(final AngularSpeedTriad bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        biasX = convertAngularSpeed(bias.getValueX(), bias.getUnit());
        biasY = convertAngularSpeed(bias.getValueY(), bias.getUnit());
        biasZ = convertAngularSpeed(bias.getValueZ(), bias.getUnit());
    }

    /**
     * Gets initial x scaling factor of gyroscope.
     *
     * @return initial x scaling factor of gyroscope.
     */
    @Override
    public double getInitialSx() {
        return initialSx;
    }

    /**
     * Sets initial x scaling factor of gyroscope.
     *
     * @param initialSx initial x scaling factor of gyroscope.
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
     * Gets initial y scaling factor of gyroscope.
     *
     * @return initial y scaling factor of gyroscope.
     */
    @Override
    public double getInitialSy() {
        return initialSy;
    }

    /**
     * Sets initial y scaling factor of gyroscope.
     *
     * @param initialSy initial y scaling factor of gyroscope.
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
     * Gets initial z scaling factor of gyroscope.
     *
     * @return initial z scaling factor of gyroscope.
     */
    @Override
    public double getInitialSz() {
        return initialSz;
    }

    /**
     * Sets initial z scaling factor of gyroscope.
     *
     * @param initialSz initial z scaling factor of gyroscope.
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
     * Gets initial x-y cross coupling error of gyroscope.
     *
     * @return initial x-y cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMxy() {
        return initialMxy;
    }

    /**
     * Sets initial x-y cross coupling error of gyroscope.
     *
     * @param initialMxy initial x-y cross coupling error of gyroscope.
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
     * Gets initial x-z cross coupling error of gyroscope.
     *
     * @return initial x-z cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMxz() {
        return initialMxz;
    }

    /**
     * Sets initial x-z cross coupling error of gyroscope.
     *
     * @param initialMxz initial x-z cross coupling error of gyroscope.
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
     * Gets initial y-x cross coupling error of gyroscope.
     *
     * @return initial y-x cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMyx() {
        return initialMyx;
    }

    /**
     * Sets initial y-x cross coupling error of gyroscope.
     *
     * @param initialMyx initial y-x cross coupling error of gyroscope.
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
     * Gets initial y-z cross coupling error of gyroscope.
     *
     * @return initial y-z cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMyz() {
        return initialMyz;
    }

    /**
     * Sets initial y-z cross coupling error of gyroscope.
     *
     * @param initialMyz initial y-z cross coupling error of gyroscope.
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
     * Gets initial z-x cross coupling error of gyroscope.
     *
     * @return initial z-x cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMzx() {
        return initialMzx;
    }

    /**
     * Sets initial z-x cross coupling error of gyroscope.
     *
     * @param initialMzx initial z-x cross coupling error of gyroscope.
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
     * Gets initial z-y cross coupling error of gyroscope.
     *
     * @return initial z-y cross coupling error of gyroscope.
     */
    @Override
    public double getInitialMzy() {
        return initialMzy;
    }

    /**
     * Sets initial z-y cross coupling error of gyroscope.
     *
     * @param initialMzy initial z-y cross coupling error of gyroscope.
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
        if (running) {
            throw new LockedException();
        }
        this.initialSx = initialSx;
        this.initialSy = initialSy;
        this.initialSz = initialSz;
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
        if (running) {
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
        final var result = new double[BodyKinematics.COMPONENTS];
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
        result[0] = biasX;
        result[1] = biasY;
        result[2] = biasZ;
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
        if (running) {
            throw new LockedException();
        }

        if (bias.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        biasX = bias[0];
        biasY = bias[1];
        biasZ = bias[2];
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
        result.setElementAtIndex(0, biasX);
        result.setElementAtIndex(1, biasY);
        result.setElementAtIndex(2, biasZ);
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
        if (running) {
            throw new LockedException();
        }
        if (initialBias.getRows() != BodyKinematics.COMPONENTS || initialBias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        biasX = initialBias.getElementAtIndex(0);
        biasY = initialBias.getElementAtIndex(1);
        biasZ = initialBias.getElementAtIndex(2);
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
     * Sets initial gyroscope scale factors and cross coupling errors matrix.
     *
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setInitialMg(final Matrix initialMg) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (initialMg.getRows() != BodyKinematics.COMPONENTS || initialMg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialSx = initialMg.getElementAtIndex(0);
        initialMyx = initialMg.getElementAtIndex(1);
        initialMzx = initialMg.getElementAtIndex(2);

        initialMxy = initialMg.getElementAtIndex(3);
        initialSy = initialMg.getElementAtIndex(4);
        initialMzy = initialMg.getElementAtIndex(5);

        initialMxz = initialMg.getElementAtIndex(6);
        initialMyz = initialMg.getElementAtIndex(7);
        initialSz = initialMg.getElementAtIndex(8);
    }

    /**
     * Gets initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing initial g-dependent cross biases.
     */
    @Override
    public Matrix getInitialGg() {
        return new Matrix(initialGg);
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

        result.copyFrom(initialGg);
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
        if (running) {
            throw new LockedException();
        }

        if (initialGg.getRows() != BodyKinematics.COMPONENTS || initialGg.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialGg.copyTo(this.initialGg);
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
        return sequences;
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
        if (running) {
            throw new LockedException();
        }
        this.sequences = sequences;
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
     * Indicates whether G-dependent cross biases are being estimated
     * or not.
     * When enabled, this adds 9 variables from Gg matrix.
     *
     * @return true if G-dependent cross biases will be estimated,
     * false otherwise.
     */
    public boolean isGDependentCrossBiasesEstimated() {
        return estimateGDependentCrossBiases;
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
        if (running) {
            throw new LockedException();
        }

        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RobustKnownBiasEasyGyroscopeCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownBiasEasyGyroscopeCalibratorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets minimum number of required sequences.
     *
     * @return minimum number of required sequences.
     */
    @Override
    public int getMinimumRequiredMeasurementsOrSequences() {
        if (commonAxisUsed) {
            if (estimateGDependentCrossBiases) {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            } else {
                return KnownBiasEasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            }
        } else {
            if (estimateGDependentCrossBiases) {
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
        return sequences != null && sequences.size() >= getMinimumRequiredMeasurementsOrSequences();
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
        return estimatedMg;
    }

    /**
     * Gets estimated gyroscope x-axis scale factor.
     *
     * @return estimated gyroscope x-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-axis scale factor.
     *
     * @return estimated gyroscope y-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated gyroscope z-axis scale factor.
     *
     * @return estimated gyroscope z-axis scale factor or null
     * if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return estimatedMg != null ? estimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated gyroscope x-y cross-coupling error.
     *
     * @return estimated gyroscope x-y cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated gyroscope x-z cross-coupling error.
     *
     * @return estimated gyroscope x-z cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated gyroscope y-x cross-coupling error.
     *
     * @return estimated gyroscope y-x cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated gyroscope y-z cross-coupling error.
     *
     * @return estimated gyroscope y-z cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated gyroscope z-x cross-coupling error.
     *
     * @return estimated gyroscope z-x cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return estimatedMg != null ? estimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated gyroscope z-y cross-coupling error.
     *
     * @return estimated gyroscope z-y cross-coupling error or null
     * if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return estimatedMg != null ? estimatedMg.getElementAt(2, 1) : null;
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
        return estimatedGg;
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
     * This has to be at least {@link #getMinimumRequiredMeasurementsOrSequences()}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return preliminarySubsetSize;
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
        if (running) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < getMinimumRequiredMeasurementsOrSequences()) {
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
        accelerationFixer.setBias(getAccelerometerBias());
        accelerationFixer.setCrossCouplingErrors(getAccelerometerMa());
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
            angularRateFixer.setBias(biasX, biasY, biasZ);
            angularRateFixer.setCrossCouplingErrors(preliminaryResult.estimatedMg);
            angularRateFixer.setGDependantCrossBias(preliminaryResult.estimatedGg);

            // copy measured sequence as it will be used to fix kinematics values
            // using preliminary gyroscope parameters
            final var fixedSequence = new BodyKinematicsSequence<>(sequence);

            // fix body kinematic measurements of provided sequence
            final var numItems = sequence.getItemsCount();
            final var measuredItems = sequence.getSortedItems();
            final var fixedItems = fixedSequence.getSortedItems();
            for (var j = 0; j < numItems; j++) {
                final var measuredItem = measuredItems.get(j);
                final var fixedItem = fixedItems.get(j);

                final var measuredKinematics = measuredItem.getKinematics();
                final var fixedKinematics = fixedItem.getKinematics();
                fixKinematics(measuredKinematics, fixedKinematics);
            }

            // integrate fixed sequence to obtain attitude change
            QuaternionIntegrator.integrateGyroSequence(fixedSequence, QuaternionStepIntegratorType.RUNGE_KUTTA, q);

            // fix before coordinates
            measuredSpecificForce[0] = sequence.getBeforeMeanFx();
            measuredSpecificForce[1] = sequence.getBeforeMeanFy();
            measuredSpecificForce[2] = sequence.getBeforeMeanFz();
            accelerationFixer.fix(measuredSpecificForce, fixedSpecificForce);

            // normalize coordinates
            ArrayUtils.normalize(fixedSpecificForce);

            // compute estimated normalized end coordinates
            startPoint.setCoordinates(fixedSpecificForce);
            q.inverse();
            q.rotate(startPoint, endPoint);

            // fix after coordinates
            measuredSpecificForce[0] = sequence.getAfterMeanFx();
            measuredSpecificForce[1] = sequence.getAfterMeanFy();
            measuredSpecificForce[2] = sequence.getAfterMeanFz();
            accelerationFixer.fix(measuredSpecificForce, fixedSpecificForce);

            // normalize coordinates
            ArrayUtils.normalize(fixedSpecificForce);

            expectedEndPoint.setCoordinates(fixedSpecificForce);

            // compare estimated normalized end coordinates with expected
            // ones
            return expectedEndPoint.distanceTo(endPoint);

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
        final var seqs = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();

        for (var samplesIndex : samplesIndices) {
            seqs.add(this.sequences.get(samplesIndex));
        }

        try {
            final var result = new PreliminaryResult();
            result.estimatedMg = getInitialMg();
            result.estimatedGg = getInitialGg();

            innerCalibrator.setGDependentCrossBiasesEstimated(estimateGDependentCrossBiases);
            innerCalibrator.setBiasCoordinates(biasX, biasY, biasZ);
            innerCalibrator.setInitialMg(result.estimatedMg);
            innerCalibrator.setInitialGg(result.estimatedGg);
            innerCalibrator.setAccelerometerBias(accelerometerBiasX, accelerometerBiasY, accelerometerBiasZ);
            innerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                    accelerometerSx, accelerometerSy, accelerometerSz,
                    accelerometerMxy, accelerometerMxz, accelerometerMyx,
                    accelerometerMyz, accelerometerMzx, accelerometerMzy);
            innerCalibrator.setCommonAxisUsed(commonAxisUsed);
            innerCalibrator.setSequences(seqs);
            innerCalibrator.calibrate();

            result.estimatedMg = innerCalibrator.getEstimatedMg();
            result.estimatedGg = innerCalibrator.getEstimatedGg();

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
            final var nSamples = sequences.size();

            final var inlierSequences = new ArrayList<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>();
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierSequences.add(sequences.get(i));
                }
            }

            try {
                innerCalibrator.setGDependentCrossBiasesEstimated(estimateGDependentCrossBiases);
                innerCalibrator.setBiasCoordinates(biasX, biasY, biasZ);
                innerCalibrator.setInitialMg(preliminaryResult.estimatedMg);
                innerCalibrator.setInitialGg(preliminaryResult.estimatedGg);
                innerCalibrator.setAccelerometerBias(accelerometerBiasX, accelerometerBiasY, accelerometerBiasZ);
                innerCalibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                        accelerometerSx, accelerometerSy, accelerometerSz,
                        accelerometerMxy, accelerometerMxz, accelerometerMyx,
                        accelerometerMyz, accelerometerMzx, accelerometerMzy);
                innerCalibrator.setCommonAxisUsed(commonAxisUsed);
                innerCalibrator.setSequences(inlierSequences);
                innerCalibrator.calibrate();

                estimatedMg = innerCalibrator.getEstimatedMg();
                estimatedGg = innerCalibrator.getEstimatedGg();

                if (keepCovariance) {
                    estimatedCovariance = innerCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }

                estimatedMse = innerCalibrator.getEstimatedMse();
                estimatedChiSq = innerCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = preliminaryResult.covariance;
                estimatedMg = preliminaryResult.estimatedMg;
                estimatedGg = preliminaryResult.estimatedGg;
                estimatedMse = preliminaryResult.estimatedMse;
                estimatedChiSq = preliminaryResult.estimatedChiSq;
            }
        } else {
            estimatedCovariance = preliminaryResult.covariance;
            estimatedMg = preliminaryResult.estimatedMg;
            estimatedGg = preliminaryResult.estimatedGg;
            estimatedMse = preliminaryResult.estimatedMse;
            estimatedChiSq = preliminaryResult.estimatedChiSq;
        }
    }

    /**
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    private static double convertAcceleration(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(), acceleration.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
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

        measuredSpecificForce[0] = measuredKinematics.getFx();
        measuredSpecificForce[1] = measuredKinematics.getFy();
        measuredSpecificForce[2] = measuredKinematics.getFz();
        accelerationFixer.fix(measuredSpecificForce, fixedSpecificForce);

        measuredAngularRate[0] = measuredKinematics.getAngularRateX();
        measuredAngularRate[1] = measuredKinematics.getAngularRateY();
        measuredAngularRate[2] = measuredKinematics.getAngularRateZ();
        angularRateFixer.fix(measuredAngularRate, fixedSpecificForce, fixedAngularRate);

        result.setSpecificForceCoordinates(fixedSpecificForce[0], fixedSpecificForce[1], fixedSpecificForce[2]);
        result.setAngularRateCoordinates(fixedAngularRate[0], fixedAngularRate[1], fixedAngularRate[2]);
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
        private Matrix estimatedMg;

        /**
         * Estimated G-dependent cross biases introduced on the gyroscope by the
         * specific forces sensed by the accelerometer.
         * This instance allows any 3x3 matrix.
         */
        private Matrix estimatedGg;

        /**
         * Covariance matrix for estimated result.
         */
        private Matrix covariance;

        /**
         * Estimated Mean Square Error.
         */
        private double estimatedMse;

        /**
         * Estimated chi square value.
         */
        private double estimatedChiSq;
    }
}
