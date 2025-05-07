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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate magnetometer
 * soft-iron cross couplings and scaling factors.
 * <p>
 * To use this calibrator at least 3 measurements at different known
 * frames must be provided. In other words, magnetometer samples must
 * be obtained at 3 different positions or orientations.
 * Notice that frame velocities are ignored by this calibrator.
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
 */
public abstract class RobustKnownHardIronAndFrameMagnetometerCalibrator implements MagnetometerNonLinearCalibrator,
        KnownHardIronMagnetometerCalibrator, OrderedStandardDeviationFrameBodyMagneticFluxDensityMagnetometerCalibrator,
        QualityScoredMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 3;

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
     * Contains a list of body magnetic flux density measurements taken
     * at different frames (positions and orientations) and containing the
     * standard deviation of magnetometer measurements.
     * If a single device magnetometer needs to be calibrated, typically all
     * measurements are taken at the same position, with zero velocity and
     * multiple orientations.
     * However, if we just want to calibrate a given magnetometer model (e.g.
     * obtain an average and less precise calibration for the magnetometer of
     * a given phone model), we could take measurements collected throughout
     * the planet at multiple positions while the phone remains static (e.g.
     * while charging), hence each measurement position will change, velocity
     * will remain zero and orientation will be typically constant at
     * horizontal orientation while the phone remains on a
     * flat surface.
     */
    protected List<StandardDeviationFrameBodyMagneticFluxDensity> measurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener;

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
     * Data related to inlier found after calibration.
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
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from soft-iron (Mm) matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * X-coordinate of known hard-iron bias.
     * This is expressed in Teslas (T).
     */
    private double hardIronX;

    /**
     * Y-coordinate of known hard-iron bias.
     * This is expressed in Teslas (T).
     */
    private double hardIronY;

    /**
     * Z-coordinate of known hard-iron bias.
     * This is expressed in Teslas (T).
     */
    private double hardIronZ;

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
     * Estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix estimatedMm;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean keepCovariance = DEFAULT_KEEP_COVARIANCE;

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
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel magneticModel;

    /**
     * A linear least squares calibrator.
     */
    private final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator linearCalibrator =
            new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

    /**
     * A non-linear least squares calibrator.
     */
    private final KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator nonLinearCalibrator =
            new KnownHardIronAndFrameMagnetometerNonLinearLeastSquaresCalibrator();

    /**
     * World Magnetic Model estimator.
     */
    private WMMEarthMagneticFluxDensityEstimator wmmEstimator;

    /**
     * Constructor.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final boolean commonAxisUsed, final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownHardIronAndFrameMagnetometerCalibrator(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Gets x-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return x-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronX() {
        return hardIronX;
    }

    /**
     * Sets x-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronX x coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronX(final double hardIronX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronX = hardIronX;
    }

    /**
     * Gets y-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return y-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronY() {
        return hardIronY;
    }

    /**
     * Sets y-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronY y coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronY(final double hardIronY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronY = hardIronY;
    }

    /**
     * Gets z-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return z-coordinate of known magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronZ() {
        return hardIronZ;
    }

    /**
     * Sets z-coordinate of known magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronZ z coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronZ(final double hardIronZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronZ = hardIronZ;
    }

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @return x coordinate of magnetometer hard-iron.
     */
    @Override
    public MagneticFluxDensity getHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(hardIronX, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known x coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(hardIronX);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known x-coordinate of magnetometer hard-iron.
     *
     * @param hardIronX known x-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronX(final MagneticFluxDensity hardIronX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronX = convertMagneticFluxDensity(hardIronX);
    }

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @return y coordinate of magnetometer hard-iron.
     */
    @Override
    public MagneticFluxDensity getHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(hardIronY, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known y coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(hardIronY);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known y-coordinate of magnetometer hard-iron.
     *
     * @param hardIronY known y-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronY(final MagneticFluxDensity hardIronY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronY = convertMagneticFluxDensity(hardIronY);
    }

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @return z coordinate of magnetometer hard-iron.
     */
    @Override
    public MagneticFluxDensity getHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(hardIronZ, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets known z coordinate of magnetometer hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(hardIronZ);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known z-coordinate of magnetometer hard-iron.
     *
     * @param hardIronZ known z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronZ(final MagneticFluxDensity hardIronZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronZ = convertMagneticFluxDensity(hardIronZ);
    }

    /**
     * Sets known hard-iron bias coordinates of magnetometer expressed
     * in Teslas (T).
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronCoordinates(
            final double hardIronX, final double hardIronY, final double hardIronZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronX = hardIronX;
        this.hardIronY = hardIronY;
        this.hardIronZ = hardIronZ;
    }

    /**
     * Sets known hard-iron coordinates.
     *
     * @param hardIronX x-coordinate of magnetometer hard-iron.
     * @param hardIronY y-coordinate of magnetometer hard-iron.
     * @param hardIronZ z-coordinate of magnetometer hard-iron.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIronCoordinates(
            final MagneticFluxDensity hardIronX, final MagneticFluxDensity hardIronY,
            final MagneticFluxDensity hardIronZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.hardIronX = convertMagneticFluxDensity(hardIronX);
        this.hardIronY = convertMagneticFluxDensity(hardIronY);
        this.hardIronZ = convertMagneticFluxDensity(hardIronZ);
    }

    /**
     * Gets known hard-iron.
     *
     * @return known hard-iron.
     */
    @Override
    public MagneticFluxDensityTriad getHardIronAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, hardIronX, hardIronY, hardIronZ);
    }

    /**
     * Gets known hard-iron.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getHardIronAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(hardIronX, hardIronY, hardIronZ, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets known hard-iron.
     *
     * @param hardIron hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setHardIron(final MagneticFluxDensityTriad hardIron) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        hardIronX = convertMagneticFluxDensity(hardIron.getValueX(), hardIron.getUnit());
        hardIronY = convertMagneticFluxDensity(hardIron.getValueY(), hardIron.getUnit());
        hardIronZ = convertMagneticFluxDensity(hardIron.getValueZ(), hardIron.getUnit());
    }

    /**
     * Gets initial x scaling factor.
     *
     * @return initial x scaling factor.
     */
    @Override
    public double getInitialSx() {
        return initialSx;
    }

    /**
     * Sets initial x scaling factor.
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
     *
     * @return initial y scaling factor.
     */
    @Override
    public double getInitialSy() {
        return initialSy;
    }

    /**
     * Sets initial y scaling factor.
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
     *
     * @return initial z scaling factor.
     */
    @Override
    public double getInitialSz() {
        return initialSz;
    }

    /**
     * Sets initial z scaling factor.
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
     *
     * @return initial x-y cross coupling error.
     */
    @Override
    public double getInitialMxy() {
        return initialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
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
     *
     * @return initial x-z cross coupling error.
     */
    @Override
    public double getInitialMxz() {
        return initialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
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
     *
     * @return initial y-x cross coupling error.
     */
    @Override
    public double getInitialMyx() {
        return initialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
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
     *
     * @return initial y-z cross coupling error.
     */
    @Override
    public double getInitialMyz() {
        return initialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
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
     *
     * @return initial z-x cross coupling error.
     */
    @Override
    public double getInitialMzx() {
        return initialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
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
     *
     * @return initial z-y cross coupling error.
     */
    @Override
    public double getInitialMzy() {
        return initialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
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
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getHardIron() {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        getHardIron(result);
        return result;
    }

    /**
     * Gets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void getHardIron(final double[] result) {
        if (result.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = hardIronX;
        result[1] = hardIronY;
        result[2] = hardIronZ;
    }

    /**
     * Sets known hard-iron bias as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param hardIron known hard-iron bias.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void setHardIron(final double[] hardIron) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (hardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        hardIronX = hardIron[0];
        hardIronY = hardIron[1];
        hardIronZ = hardIron[2];
    }

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @return hard-iron bias as a column matrix.
     */
    @Override
    public Matrix getHardIronMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
            getHardIronMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets known hard-iron bias as a column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getHardIronMatrix(final Matrix result) {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, hardIronX);
        result.setElementAtIndex(1, hardIronY);
        result.setElementAtIndex(2, hardIronZ);
    }

    /**
     * Sets known hard-iron bias.
     *
     * @param hardIron magnetometer hard-iron bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setHardIron(final Matrix hardIron) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (hardIron.getRows() != BodyMagneticFluxDensity.COMPONENTS || hardIron.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        hardIronX = hardIron.getElementAtIndex(0);
        hardIronY = hardIron.getElementAtIndex(1);
        hardIronZ = hardIron.getElementAtIndex(2);
    }

    /**
     * Gets initial scale factors and cross coupling errors matrix.
     *
     * @return initial scale factors and cross coupling errors matrix.
     */
    @Override
    public Matrix getInitialMm() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            getInitialMm(result);
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
    public void getInitialMm(final Matrix result) {
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
     * @param initialMm initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setInitialMm(final Matrix initialMm) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (initialMm.getRows() != BodyKinematics.COMPONENTS || initialMm.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        initialSx = initialMm.getElementAtIndex(0);
        initialMyx = initialMm.getElementAtIndex(1);
        initialMzx = initialMm.getElementAtIndex(2);

        initialMxy = initialMm.getElementAtIndex(3);
        initialSy = initialMm.getElementAtIndex(4);
        initialMzy = initialMm.getElementAtIndex(5);

        initialMxz = initialMm.getElementAtIndex(6);
        initialMyz = initialMm.getElementAtIndex(7);
        initialSz = initialMm.getElementAtIndex(8);
    }

    /**
     * Gets a list of body magnetic flux density measurements taken at different
     * frames (positions, orientations and velocities).
     * If a single device IMU needs to be calibrated, typically all measurements are
     * taken at the same position, with zero velocity and multiple orientations.
     * However, if we just want to calibrate a given IMU model (e.g. obtain
     * an average and less precise calibration for the IMU of a given phone model),
     * we could take measurements collected throughout the planet at multiple positions
     * while the phone remains static (e.g. while charging), hence each measurement
     * position will change, velocity will remain zero and orientation will be
     * typically constant at horizontal orientation while the phone remains on a
     * flat surface.
     *
     * @return a collection of body magnetic flux density measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public List<StandardDeviationFrameBodyMagneticFluxDensity> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a list of body magnetic flux density measurements taken at different
     * frames (positions, orientations and velocities).
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
     * @param measurements collection of body magnetic flux density measurements
     *                     taken at different frames (positions, orientations
     *                     and velocities).
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setMeasurements(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements) throws LockedException {
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
    public MagnetometerCalibratorMeasurementType getMeasurementType() {
        return MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY;
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
     * Indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm (soft-iron) matrix.
     *
     * @return true if z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer, false otherwise.
     */
    @Override
    public boolean isCommonAxisUsed() {
        return commonAxisUsed;
    }

    /**
     * Specifies whether z-axis is assumed to be common for accelerometer and
     * gyroscope.
     * When enabled, this eliminates 3 variables from Mm matrix.
     *
     * @param commonAxisUsed true if z-axis is assumed to be common for
     *                       accelerometer, gyroscope and magnetometer, false
     *                       otherwise.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setCommonAxisUsed(final boolean commonAxisUsed) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    public RobustKnownHardIronAndFrameMagnetometerCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) throws LockedException {
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
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return measurements != null && measurements.size() >= MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is currently running or no.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return running;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    public WorldMagneticModel getMagneticModel() {
        return magneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMagneticModel(final WorldMagneticModel magneticModel) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.magneticModel = magneticModel;
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
     * Gets estimated magnetometer soft-iron matrix containing scale factors
     * and cross coupling errors.
     * This is the product of matrix Tm containing cross coupling errors and Km
     * containing scaling factors.
     * So tat:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix however, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated magnetometer soft-iron scale factors and cross coupling errors,
     * or null if not available.
     */
    @Override
    public Matrix getEstimatedMm() {
        return estimatedMm;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return estimatedMm != null ? estimatedMm.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return estimatedMm != null ? estimatedMm.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return estimatedMm != null ? estimatedMm.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return estimatedMm != null ? estimatedMm.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return estimatedMm != null ? estimatedMm.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return estimatedMm != null ? estimatedMm.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return estimatedMm != null ? estimatedMm.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return estimatedMm != null ? estimatedMm.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return estimatedMm != null ? estimatedMm.getElementAt(2, 1) : null;
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy.
     *
     * @return estimated covariance matrix for estimated position.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return estimatedCovariance;
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
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator();
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @param method   robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @param method       robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param method        robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator();
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @param method        robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density measurements with standard
     *                      deviations taken at different frames (positions and
     *                      orientations).
     * @param method        robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density measurements with standard
     *                      deviations taken at different frames (positions and
     *                      orientations).
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements,
                    listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements,
                    listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, commonAxisUsed);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, commonAxisUsed,
                    listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, commonAxisUsed,
                    listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(qualityScores, measurements,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust known frame magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param measurements list of body magnetic flux density measurements with standard
     *                     deviations taken at different frames (positions and
     *                     orientations).
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        return create(commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust known frame magnetometer calibrator using default robust method.
     *
     * @param measurements   list of body magnetic flux density measurements with standard
     *                       deviations taken at different frames (positions and
     *                       orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust known frame magnetometer calibrator.
     */
    public static RobustKnownHardIronAndFrameMagnetometerCalibrator create(
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronAndFrameMagnetometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Setups World Magnetic Model estimator.
     *
     * @throws IOException if model cannot be loaded.
     */
    protected void setupWmmEstimator() throws IOException {
        if (magneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(magneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationFrameBodyMagneticFluxDensity measurement, final Matrix preliminaryResult) {
        // The magnetometer model is:
        // mBmeas = ba + (I + Mm) * mBtrue

        // Hence:
        // [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz]) [mBtruex]
        // [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]  [mBtruey]
        // [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]  [mBtruez]

        final var measuredMagneticFluxDensity = measurement.getMagneticFluxDensity();
        final var ecefFrame = measurement.getFrame();

        final var nedFrame = ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(ecefFrame);
        final var year = measurement.getYear();

        final var latitude = nedFrame.getLatitude();
        final var longitude = nedFrame.getLongitude();
        final var height = nedFrame.getHeight();

        final var earthB = wmmEstimator.estimate(latitude, longitude, height, year);

        final var cbn = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var cnb = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);

        nedFrame.getCoordinateTransformation(cbn);
        cbn.inverse(cnb);

        final var expectedMagneticFluxDensity = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var bMeasX1 = measuredMagneticFluxDensity.getBx();
        final var bMeasY1 = measuredMagneticFluxDensity.getBy();
        final var bMeasZ1 = measuredMagneticFluxDensity.getBz();

        final var bTrueX = expectedMagneticFluxDensity.getBx();
        final var bTrueY = expectedMagneticFluxDensity.getBy();
        final var bTrueZ = expectedMagneticFluxDensity.getBz();

        try {
            final var m = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            m.add(preliminaryResult);

            final var btrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            btrue.setElementAtIndex(0, bTrueX);
            btrue.setElementAtIndex(1, bTrueY);
            btrue.setElementAtIndex(2, bTrueZ);

            m.multiply(btrue);

            final var bMeasX2 = hardIronX + m.getElementAtIndex(0);
            final var bMeasY2 = hardIronY + m.getElementAtIndex(1);
            final var bMeasZ2 = hardIronZ + m.getElementAtIndex(2);

            final var diffX = bMeasX2 - bMeasX1;
            final var diffY = bMeasY2 - bMeasY1;
            final var diffZ = bMeasZ2 - bMeasZ1;

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

        final var meas = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();

        for (final var samplesIndex : samplesIndices) {
            meas.add(this.measurements.get(samplesIndex));
        }

        try {
            final var result = getInitialMm();

            if (useLinearCalibrator) {
                linearCalibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
                linearCalibrator.setCommonAxisUsed(commonAxisUsed);
                linearCalibrator.setMeasurements(meas);
                linearCalibrator.calibrate();

                result.copyFrom(linearCalibrator.getEstimatedMm());
            }

            if (refinePreliminarySolutions) {
                nonLinearCalibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
                nonLinearCalibrator.setInitialMm(result);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(meas);
                nonLinearCalibrator.calibrate();

                result.copyFrom(nonLinearCalibrator.getEstimatedMm());
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

            final var inlierMeasurements = new ArrayList<StandardDeviationFrameBodyMagneticFluxDensity>();
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(measurements.get(i));
                }
            }

            try {
                nonLinearCalibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
                nonLinearCalibrator.setInitialMm(preliminaryResult);
                nonLinearCalibrator.setCommonAxisUsed(commonAxisUsed);
                nonLinearCalibrator.setMeasurements(inlierMeasurements);
                nonLinearCalibrator.calibrate();

                estimatedMm = nonLinearCalibrator.getEstimatedMm();

                if (keepCovariance) {
                    estimatedCovariance = nonLinearCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }

                estimatedMse = nonLinearCalibrator.getEstimatedMse();
                estimatedChiSq = nonLinearCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = null;
                estimatedMm = preliminaryResult;
                estimatedMse = 0.0;
                estimatedChiSq = 0.0;
            }
        } else {
            estimatedCovariance = null;
            estimatedMm = preliminaryResult;
            estimatedMse = 0.0;
            estimatedChiSq = 0.0;
        }
    }

    /**
     * Converts magnetic flux density value and unit to Teslas.
     *
     * @param value magnetic flux density value.
     * @param unit  unit of magnetic flux density value.
     * @return converted value.
     */
    private static double convertMagneticFluxDensity(final double value, final MagneticFluxDensityUnit unit) {
        return MagneticFluxDensityConverter.convert(value, unit, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Converts magnetic flux density instance to Teslas.
     *
     * @param magneticFluxDensity magnetic flux density instance to be converted.
     * @return converted value.
     */
    private static double convertMagneticFluxDensity(final MagneticFluxDensity magneticFluxDensity) {
        return convertMagneticFluxDensity(magneticFluxDensity.getValue().doubleValue(), magneticFluxDensity.getUnit());
    }
}
