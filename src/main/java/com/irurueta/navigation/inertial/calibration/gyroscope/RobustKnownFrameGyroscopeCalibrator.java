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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.GyroscopeBiasUncertaintySource;
import com.irurueta.navigation.inertial.calibration.GyroscopeCalibrationSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate gyroscope
 * biases, cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * To use this calibrator at least 7 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 7 different positions, orientations and velocities
 * (although typically velocities are always zero).
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
public abstract class RobustKnownFrameGyroscopeCalibrator implements
        GyroscopeNonLinearCalibrator, UnknownBiasNonLinearGyroscopeCalibrator, GyroscopeCalibrationSource,
        GyroscopeBiasUncertaintySource, OrderedStandardDeviationFrameBodyKinematicsGyroscopeCalibrator,
        QualityScoredGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 7;

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
    protected RobustKnownFrameGyroscopeCalibratorListener listener;

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
     * Initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double initialBiasX;

    /**
     * Initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
     */
    private double initialBiasY;

    /**
     * Initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s).
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
     * Initial G-dependent cross biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     */
    private Matrix initialGg;

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
     * Estimated gyroscope biases for each IMU axis expressed in radians per second
     * (rad/s).
     */
    private double[] estimatedBiases;

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
    private final KnownFrameGyroscopeLinearLeastSquaresCalibrator linearCalibrator =
            new KnownFrameGyroscopeLinearLeastSquaresCalibrator();

    /**
     * A non-linear least squares calibrator.
     */
    private final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator nonLinearCalibrator =
            new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

    /**
     * Constructor.
     */
    protected RobustKnownFrameGyroscopeCalibrator() {
        try {
            initialGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownFrameGyroscopeCalibrator(final RobustKnownFrameGyroscopeCalibratorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    protected RobustKnownFrameGyroscopeCalibrator(final List<StandardDeviationFrameBodyKinematics> measurements) {
        this();
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to be notified of events such as when estimation
     *                     starts, ends or its progress significantly changes.
     */
    protected RobustKnownFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected RobustKnownFrameGyroscopeCalibrator(final boolean commonAxisUsed) {
        this();
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownFrameGyroscopeCalibrator(
            final boolean commonAxisUsed, final RobustKnownFrameGyroscopeCalibratorListener listener) {
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
    protected RobustKnownFrameGyroscopeCalibrator(
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
    protected RobustKnownFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solutions.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasX() {
        return initialBiasX;
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
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
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasY() {
        return initialBiasY;
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
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
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    @Override
    public double getInitialBiasZ() {
        return initialBiasZ;
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     * This is expressed in radians per second (rad/s) and only taken into
     * account if non-linear preliminary solutions are used.
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
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
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedX() {
        return new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(initialBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasX(final AngularSpeed initialBiasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = convertAngularSpeed(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedY() {
        return new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(initialBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasY(final AngularSpeed initialBiasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasY = convertAngularSpeed(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getInitialBiasAngularSpeedZ() {
        return new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getInitialBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(initialBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a solution.
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBiasZ(final AngularSpeed initialBiasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasZ = convertAngularSpeed(initialBiasZ);
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution
     * expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
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
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(
            final AngularSpeed initialBiasX, final AngularSpeed initialBiasY, final AngularSpeed initialBiasZ)
            throws LockedException {

        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = convertAngularSpeed(initialBiasX);
        this.initialBiasY = convertAngularSpeed(initialBiasY);
        this.initialBiasZ = convertAngularSpeed(initialBiasZ);
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
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in radians per second (rad/s).
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
     * Array values are expressed in radians per second (rad/s).
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
     * Array values are expressed in radians per second (rad/s).
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
     * Values are expressed in radians per second (rad/s).
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
     * Values are expressed in radians per second (rad/s).
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
     * Sets initial bias to be used to find a solution as a column matrix
     * with values expressed in radians per second (rad/s).
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
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @return initial bias coordinates.
     */
    @Override
    public AngularSpeedTriad getInitialBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialBiasAsTriad(final AngularSpeedTriad result) {
        result.setValueCoordinatesAndUnit(initialBiasX, initialBiasY, initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBias initial bias coordinates to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(final AngularSpeedTriad initialBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        initialBiasX = convertAngularSpeed(initialBias.getValueX(), initialBias.getUnit());
        initialBiasY = convertAngularSpeed(initialBias.getValueY(), initialBias.getUnit());
        initialBiasZ = convertAngularSpeed(initialBias.getValueZ(), initialBias.getUnit());
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
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
     * Gets initial scale factors and cross coupling errors matrix.
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
     * Sets initial scale factors and cross coupling errors matrix.
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
     * Indicates the type of measurement or sequence used by this calibrator.
     *
     * @return type of measurement or sequence used by this calibrator.
     */
    @Override
    public GyroscopeCalibratorMeasurementOrSequenceType getMeasurementOrSequenceType() {
        return GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT;
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
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public RobustKnownFrameGyroscopeCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownFrameGyroscopeCalibratorListener listener) throws LockedException {
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
    public int getMinimumRequiredMeasurementsOrSequences() {
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
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    @Override
    public double[] getEstimatedBiases() {
        return estimatedBiases;
    }

    /**
     * Gets array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @param result instance where estimated gyroscope biases will be stored.
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
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return column matrix containing x,y,z components of estimated gyroscope
     * biases.
     */
    @Override
    public Matrix getEstimatedBiasesAsMatrix() {
        return estimatedBiases != null ? Matrix.newFromArray(estimatedBiases) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
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
     * Gets x coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasX() {
        return estimatedBiases != null ? estimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasY() {
        return estimatedBiases != null ? estimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per
     * second (rad/s).
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasZ() {
        return estimatedBiases != null ? estimatedBiases[2] : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedX() {
        return estimatedBiases != null
                ? new AngularSpeed(estimatedBiases[0], AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets x coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedX(final AngularSpeed result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[0]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedY() {
        return estimatedBiases != null
                ? new AngularSpeed(estimatedBiases[1], AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedY(final AngularSpeed result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[1]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @return z coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeed getEstimatedBiasAngularSpeedZ() {
        return estimatedBiases != null
                ? new AngularSpeed(estimatedBiases[2], AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if result was updated, false if estimation is not available.
     */
    @Override
    public boolean getEstimatedBiasAngularSpeedZ(final AngularSpeed result) {
        if (estimatedBiases != null) {
            result.setValue(estimatedBiases[2]);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated gyroscope bias.
     *
     * @return estimated gyroscope bias or null if not available.
     */
    @Override
    public AngularSpeedTriad getEstimatedBiasAsTriad() {
        return estimatedBiases != null
                ? new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                estimatedBiases[0], estimatedBiases[1], estimatedBiases[2])
                : null;
    }

    /**
     * Gets estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated gyroscope bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedBiasAsTriad(final AngularSpeedTriad result) {
        if (estimatedBiases != null) {
            result.setValueCoordinatesAndUnit(
                    estimatedBiases[0], estimatedBiases[1], estimatedBiases[2],
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
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
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return estimatedMg != null ? estimatedMg.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return estimatedMg != null ? estimatedMg.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return estimatedMg != null ? estimatedMg.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return estimatedMg != null ? estimatedMg.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
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
     * Gets estimated mean square error respect to provided measurements.
     *
     * @return estimated mean square error respect to provided measurements.
     */
    @Override
    public double getEstimatedMse() {
        return estimatedMse;
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
     * Gets estimated covariance matrix for estimated calibration solution.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): bgx, bgy, bgz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32,
     * gg13, gg23, gg33.
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
     * Gets variance of estimated x coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated x coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasXVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated x coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasXStandardDeviation() {
        final var variance = getEstimatedBiasXVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated x coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasXStandardDeviationAsAngularSpeed() {
        return estimatedCovariance != null
                ? new AngularSpeed(getEstimatedBiasXStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasXStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasXStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated y coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasYVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated y coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasYStandardDeviation() {
        final var variance = getEstimatedBiasYVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated y coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasYStandardDeviationAsAngularSpeed() {
        return estimatedCovariance != null
                ? new AngularSpeed(getEstimatedBiasYStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasYStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasYStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return variance of estimated z coordinate of gyroscope bias or null if not available.
     */
    public Double getEstimatedBiasZVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias expressed in
     * radians per second (rad/s).
     *
     * @return standard deviation of estimated z coordinate of gyroscope bias or null if not
     * available.
     */
    public Double getEstimatedBiasZStandardDeviation() {
        final var variance = getEstimatedBiasZVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias.
     *
     * @return standard deviation of estimated z coordinate of gyroscope bias or null if not
     * available.
     */
    public AngularSpeed getEstimatedBiasZStandardDeviationAsAngularSpeed() {
        return estimatedCovariance != null
                ? new AngularSpeed(getEstimatedBiasZStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of gyroscope bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasZStandardDeviationAsAngularSpeed(final AngularSpeed result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasZStandardDeviation());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated gyroscope bias coordinates.
     *
     * @return standard deviation of estimated gyroscope bias coordinates.
     */
    public AngularSpeedTriad getEstimatedBiasStandardDeviation() {
        return estimatedCovariance != null
                ? new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND,
                getEstimatedBiasXStandardDeviation(),
                getEstimatedBiasYStandardDeviation(),
                getEstimatedBiasZStandardDeviation())
                : null;
    }

    /**
     * Gets standard deviation of estimated gyroscope bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of gyroscope bias was available, false
     * otherwise.
     */
    public boolean getEstimatedBiasStandardDeviation(final AngularSpeedTriad result) {
        if (estimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedBiasXStandardDeviation(),
                    getEstimatedBiasYStandardDeviation(),
                    getEstimatedBiasZStandardDeviation(),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates expressed
     * in radians per second (rad/s).
     *
     * @return average of estimated standard deviation of gyroscope bias coordinates or null
     * if not available.
     */
    public Double getEstimatedBiasStandardDeviationAverage() {
        return estimatedCovariance != null
                ? (getEstimatedBiasXStandardDeviation() + getEstimatedBiasYStandardDeviation()
                + getEstimatedBiasZStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates.
     *
     * @return average of estimated standard deviation of gyroscope bias coordinates or null.
     */
    public AngularSpeed getEstimatedBiasStandardDeviationAverageAsAngularSpeed() {
        return estimatedCovariance != null
                ? new AngularSpeed(getEstimatedBiasStandardDeviationAverage(), AngularSpeedUnit.RADIANS_PER_SECOND)
                : null;
    }

    /**
     * Gets average of estimated standard deviation of gyroscope bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of gyroscope bias is available,
     * false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationAverageAsAngularSpeed(final AngularSpeed result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationAverage());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias expressed in
     * radians per second (rad/s).
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    @Override
    public Double getEstimatedBiasStandardDeviationNorm() {
        return estimatedCovariance != null
                ? Math.sqrt(getEstimatedBiasXVariance() + getEstimatedBiasYVariance() + getEstimatedBiasZVariance())
                : null;
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias.
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return norm of estimated standard deviation of gyroscope bias or null
     * if not available.
     */
    public AngularSpeed getEstimatedBiasStandardDeviationNormAsAngularSpeed() {
        return estimatedCovariance != null
                ? new AngularSpeed(getEstimatedBiasStandardDeviationNorm(), AngularSpeedUnit.RADIANS_PER_SECOND)
                : null;
    }

    /**
     * Gets norm of estimated standard deviation of gyroscope bias coordinates.
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of gyroscope bias is
     * available, false otherwise.
     */
    public boolean getEstimatedBiasStandardDeviationNormAsAngularSpeed(final AngularSpeed result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedBiasStandardDeviationNorm());
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
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
    public void setPreliminarySubsetSize(final int preliminarySubsetSize)
            throws LockedException {
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
     * Creates a robust gyroscope calibrator.
     *
     * @param method robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator();
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator();
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator();
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator();
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator();
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final RobustKnownFrameGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(measurements);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(measurements);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(measurements, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed, final RobustKnownFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientation
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownFrameGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator();
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator();
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator();
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final RobustKnownFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, commonAxisUsed);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, commonAxisUsed);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed,
            final RobustKnownFrameGyroscopeCalibratorListener listener, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
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
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust gyroscope calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better the
     *                       quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownFrameGyroscopeCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownFrameGyroscopeCalibrator(measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, commonAxisUsed,
                    listener);
            default -> new PROMedSRobustKnownFrameGyroscopeCalibrator(qualityScores, measurements, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust accelerometer calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final boolean commonAxisUsed, final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different (positions, orientations and
     *                       velocities).
     * @param commonAxisUsed indicates whether z-axis is assigned to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(final double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(qualityScores, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements) {
        return create(qualityScores, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(qualityScores, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed) {
        return create(qualityScores, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed,
            final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(qualityScores, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        return create(qualityScores, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust gyroscope calibrator using default robust method.
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
     * @return a robust gyroscope calibrator.
     */
    public static RobustKnownFrameGyroscopeCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownFrameGyroscopeCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }


    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationFrameBodyKinematics measurement, final PreliminaryResult preliminaryResult) {
        // We know that measured angular rate is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        final var measuredKinematics = measurement.getKinematics();
        final var ecefFrame = measurement.getFrame();
        final var previousEcefFrame = measurement.getPreviousFrame();
        final var timeInterval = measurement.getTimeInterval();

        final var expectedKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                previousEcefFrame);

        final var angularRateMeasX1 = measuredKinematics.getAngularRateX();
        final var angularRateMeasY1 = measuredKinematics.getAngularRateY();
        final var angularRateMeasZ1 = measuredKinematics.getAngularRateZ();

        final var angularRateTrueX = expectedKinematics.getAngularRateX();
        final var angularRateTrueY = expectedKinematics.getAngularRateY();
        final var angularRateTrueZ = expectedKinematics.getAngularRateZ();

        final var fTrueX = expectedKinematics.getFx();
        final var fTrueY = expectedKinematics.getFy();
        final var fTrueZ = expectedKinematics.getFz();

        final var b = preliminaryResult.estimatedBiases;
        final var bx = b[0];
        final var by = b[1];
        final var bz = b[2];

        final var mg = preliminaryResult.estimatedMg;

        final var gg = preliminaryResult.estimatedGg;

        try {
            final var m1 = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            m1.add(mg);

            final var angularRateTrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            angularRateTrue.setElementAtIndex(0, angularRateTrueX);
            angularRateTrue.setElementAtIndex(1, angularRateTrueY);
            angularRateTrue.setElementAtIndex(2, angularRateTrueZ);

            m1.multiply(angularRateTrue);

            final var fTrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            fTrue.setElementAtIndex(0, fTrueX);
            fTrue.setElementAtIndex(1, fTrueY);
            fTrue.setElementAtIndex(2, fTrueZ);
            final var m2 = gg.multiplyAndReturnNew(fTrue);

            m1.add(m2);

            final var angularRateMeasX2 = bx + m1.getElementAtIndex(0);
            final var angularRateMeasY2 = by + m1.getElementAtIndex(1);
            final var angularRateMeasZ2 = bz + m1.getElementAtIndex(2);

            final var diffX = angularRateMeasX2 - angularRateMeasX1;
            final var diffY = angularRateMeasY2 - angularRateMeasY1;
            final var diffZ = angularRateMeasZ2 - angularRateMeasZ1;

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
    protected void computePreliminarySolutions(final int[] samplesIndices, final List<PreliminaryResult> solutions) {

        final var meas = new ArrayList<StandardDeviationFrameBodyKinematics>();

        for (final var samplesIndex : samplesIndices) {
            meas.add(this.measurements.get(samplesIndex));
        }

        try {
            final var result = new PreliminaryResult();
            result.estimatedBiases = getInitialBias();
            result.estimatedMg = getInitialMg();
            result.estimatedGg = getInitialGg();

            if (useLinearCalibrator) {
                linearCalibrator.setCommonAxisUsed(commonAxisUsed);
                linearCalibrator.setMeasurements(meas);
                linearCalibrator.calibrate();

                linearCalibrator.getEstimatedBiases(result.estimatedBiases);
                result.estimatedMg = linearCalibrator.getEstimatedMg();
                result.estimatedGg = linearCalibrator.getEstimatedGg();
            }

            if (refinePreliminarySolutions) {
                nonLinearCalibrator.setInitialBias(result.estimatedBiases);
                nonLinearCalibrator.setInitialMg(result.estimatedMg);
                nonLinearCalibrator.setInitialGg(result.estimatedGg);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(meas);
                nonLinearCalibrator.calibrate();

                nonLinearCalibrator.getEstimatedBiases(result.estimatedBiases);
                result.estimatedMg = nonLinearCalibrator.getEstimatedMg();
                result.estimatedGg = nonLinearCalibrator.getEstimatedGg();

                if (keepCovariance) {
                    result.covariance = nonLinearCalibrator.getEstimatedCovariance();
                } else {
                    result.covariance = null;
                }

                result.estimatedMse = nonLinearCalibrator.getEstimatedMse();
                result.estimatedChiSq = nonLinearCalibrator.getEstimatedChiSq();
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
    protected void attemptRefine(final PreliminaryResult preliminaryResult) {
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
                nonLinearCalibrator.setInitialBias(preliminaryResult.estimatedBiases);
                nonLinearCalibrator.setInitialMg(preliminaryResult.estimatedMg);
                nonLinearCalibrator.setInitialGg(preliminaryResult.estimatedGg);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(inlierMeasurements);
                nonLinearCalibrator.calibrate();

                estimatedBiases = nonLinearCalibrator.getEstimatedBiases();
                estimatedMg = nonLinearCalibrator.getEstimatedMg();
                estimatedGg = nonLinearCalibrator.getEstimatedGg();

                if (keepCovariance) {
                    estimatedCovariance = nonLinearCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }

                estimatedMse = nonLinearCalibrator.getEstimatedMse();
                estimatedChiSq = nonLinearCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = preliminaryResult.covariance;
                estimatedBiases = preliminaryResult.estimatedBiases;
                estimatedMg = preliminaryResult.estimatedMg;
                estimatedGg = preliminaryResult.estimatedGg;
                estimatedMse = preliminaryResult.estimatedMse;
                estimatedChiSq = preliminaryResult.estimatedChiSq;
            }
        } else {
            estimatedCovariance = preliminaryResult.covariance;
            estimatedBiases = preliminaryResult.estimatedBiases;
            estimatedMg = preliminaryResult.estimatedMg;
            estimatedGg = preliminaryResult.estimatedGg;
            estimatedMse = preliminaryResult.estimatedMse;
            estimatedChiSq = preliminaryResult.estimatedChiSq;
        }
    }

    /**
     * Converts angular speed value and unit to radians per second.
     *
     * @param value angular speed value.
     * @param unit  unit of angular speed value.
     * @return converted value.
     */
    private static double convertAngularSpeed(final double value, final AngularSpeedUnit unit) {
        return AngularSpeedConverter.convert(value, unit, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Converts angular speed instance to radians per second.
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value.
     */
    private static double convertAngularSpeed(final AngularSpeed angularSpeed) {
        return convertAngularSpeed(angularSpeed.getValue().doubleValue(), angularSpeed.getUnit());
    }

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
        /**
         * Estimated gyroscope biases for each IMU axis expressed in radians per second
         * (rad/s).
         */
        private double[] estimatedBiases;

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
