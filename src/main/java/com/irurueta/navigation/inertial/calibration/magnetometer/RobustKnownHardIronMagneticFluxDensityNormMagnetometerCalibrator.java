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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * This is an abstract class to robustly estimate magnetometer cross couplings
 * and scaling factors.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single unknown
 * position and instant must be taken at 7 different unknown orientations
 * when common z-axis is assumed, otherwise at least 10
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
public abstract class RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator implements
        MagnetometerNonLinearCalibrator, KnownHardIronMagnetometerCalibrator,
        OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator, QualityScoredMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS =
            BaseKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL =
            BaseKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;

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
     * Contains a list of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     */
    protected List<StandardDeviationBodyMagneticFluxDensity> measurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener;

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
    protected int preliminarySubsetSize = MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from soft-iron (Mm) matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

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
     * Known x-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double hardIronX;

    /**
     * Known y-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double hardIronY;

    /**
     * Known z-coordinate of hard-iron bias to be used to find a solution.
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
     * Ground truth magnetic flux density norm to be expected at location where measurements have been made,
     * expressed in Teslas (T).
     */
    protected Double groundTruthMagneticFluxDensityNorm;

    /**
     * Inner calibrator to compute calibration for each subset of data or during
     * final refining.
     */
    private final KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator innerCalibrator =
            new KnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();

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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this.listener = listener;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(final double[] hardIron) {
        try {
            setHardIron(hardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(final Matrix hardIron) {
        try {
            setHardIron(hardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Matrix hardIron, final Matrix initialMm) {
        this(hardIron);
        try {
            setInitialMm(initialMm);
        } catch (final LockedException ignore) {
            // never happens
        }
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this.measurements = measurements;
        this.listener = listener;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        this.measurements = measurements;
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        this(hardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, hardIron);
        this.listener = listener;
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
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        this(measurements, hardIron);
        this.commonAxisUsed = commonAxisUsed;
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        this(hardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, hardIron);
        this.listener = listener;
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
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        this(measurements, hardIron);
        this.commonAxisUsed = commonAxisUsed;
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        this(hardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, hardIron, initialMm);
        this.listener = listener;
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        this(measurements, hardIron, initialMm);
        this.commonAxisUsed = commonAxisUsed;
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(measurements, commonAxisUsed, hardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm) {
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm);
        this.listener = listener;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(groundTruthMagneticFluxDensityNorm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed) {
        this(groundTruthMagneticFluxDensityNorm);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final double[] hardIron) {
        this(hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron) {
        this(hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron, final Matrix initialMm) {
        this(hardIron, initialMm);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements);
        this.listener = listener;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        this(groundTruthMagneticFluxDensityNorm, measurements);
        this.commonAxisUsed = commonAxisUsed;
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
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        this(measurements, hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        this(measurements, commonAxisUsed, hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        this(measurements, hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        this(measurements, commonAxisUsed, hardIron);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        this(measurements, hardIron, initialMm);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
        this.listener = listener;
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        this(measurements, commonAxisUsed, hardIron, initialMm);
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    protected RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made,
     * expressed in Teslas (T).
     *
     * @return ground truth magnetic flux density or null.
     */
    public Double getGroundTruthMagneticFluxDensityNorm() {
        return groundTruthMagneticFluxDensityNorm;
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made.
     *
     * @return ground truth magnetic flux density or null.
     */
    public MagneticFluxDensity getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity() {
        return groundTruthMagneticFluxDensityNorm != null
                ? new MagneticFluxDensity(groundTruthMagneticFluxDensityNorm, MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made.
     *
     * @param result instance where result will be stored.
     * @return true if ground truth magnetic flux density norm has been defined, false if it is not available yet.
     */
    public boolean getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (groundTruthMagneticFluxDensityNorm != null) {
            result.setValue(groundTruthMagneticFluxDensityNorm);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made, expressed in Teslas (T).
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthMagneticFluxDensityNorm(final Double groundTruthMagneticFluxDensityNorm)
            throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setGroundTruthMagneticFluxDensityNorm(final MagneticFluxDensity groundTruthMagneticFluxDensityNorm)
            throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }
        if (groundTruthMagneticFluxDensityNorm != null) {
            internalSetGroundTruthMagneticFluxDensityNorm(MagneticFluxDensityConverter.convert(
                    groundTruthMagneticFluxDensityNorm.getValue().doubleValue(),
                    groundTruthMagneticFluxDensityNorm.getUnit(),
                    MagneticFluxDensityUnit.TESLA));
        } else {
            internalSetGroundTruthMagneticFluxDensityNorm(null);
        }
    }

    /**
     * Gets known x-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronX() {
        return hardIronX;
    }

    /**
     * Sets known x-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronX known x-coordinate of magnetometer
     *                  hard-iron bias.
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
     * Gets known y-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronY() {
        return hardIronY;
    }

    /**
     * Sets known y-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @param hardIronY known y-coordinate of magnetometer
     *                  hard-iron bias.
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
     * Gets known z-coordinate of magnetometer hard-iron bias.
     * This is expressed in Teslas (T).
     *
     * @return known z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getHardIronZ() {
        return hardIronZ;
    }

    /**
     * Sets known z-coordinate of magnetometer hard-iron bias.
     * This is expressed in meters Teslas (T).
     *
     * @param hardIronZ known z-coordinate of magnetometer
     *                  hard-iron bias.
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
     * Sets known hard-iron bias coordinates of magnetometer expressed in
     * Teslas (T).
     *
     * @param hardIronX x-coordinate of magnetometer
     *                  known hard-iron bias.
     * @param hardIronY y-coordinate of magnetometer
     *                  known hard-iron bias.
     * @param hardIronZ z-coordinate of magnetometer
     *                  known hard-iron bias.
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
     * @return array containing coordinates of known bias.
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
     * @param hardIron known hard-iron.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
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
     * @return known hard-iron bias as a column matrix.
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
     * Sets known hard-iron bias as a column matrix.
     *
     * @param hardIron known hard-iron bias.
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
     * Gets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @return collection of body magnetic flux density measurements at
     * a known position and timestamp with unknown orientations.
     */
    @Override
    public List<StandardDeviationBodyMagneticFluxDensity> getMeasurements() {
        return measurements;
    }

    /**
     * Sets collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements at a known position and timestamp
     *                     with unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(final List<StandardDeviationBodyMagneticFluxDensity> measurements)
            throws LockedException {
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
        return MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY;
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
    public RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener)
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
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return measurements != null && measurements.size() >= getMinimumRequiredMeasurements()
                && groundTruthMagneticFluxDensityNorm != null;
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
     * parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy.
     *
     * @return estimated covariance matrix for estimated calibration parameters.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return estimatedCovariance;
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return preliminarySubsetSize;
    }

    /**
     * Sets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     *
     * @param preliminarySubsetSize size of subsets to be checked during robust estimation.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is less than {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     */
    public void setPreliminarySubsetSize(final int preliminarySubsetSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
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
     * Creates a robust magnetometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator();
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron known hard-iron.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron known hard-iron.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @param method    robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Matrix hardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final double[] hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    measurements);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron,
                    initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron,
                    initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                    listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                    listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(measurements,
                    listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    measurements, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(qualityScores,
                    measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       known hard-iron.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       kknown hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.*
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    measurements, commonAxisUsed, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, measurements, commonAxisUsed, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3, or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm, final double[] hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1, or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    initialMm);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @param method                             robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    initialMm, listener);
            default -> new PROMedSRobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator(
                    qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                    initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(final double[] hardIron) {
        return create(hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(final Matrix hardIron) {
        return create(hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Matrix hardIron, final Matrix initialMm) {
        return create(hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        return create(measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        return create(measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        return create(measurements, commonAxisUsed, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        return create(measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        return create(measurements, commonAxisUsed, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        return create(measurements, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, hardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        return create(measurements, commonAxisUsed, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(measurements, commonAxisUsed, hardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm) {
        return create(groundTruthMagneticFluxDensityNorm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed) {
        return create(groundTruthMagneticFluxDensityNorm, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final double[] hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix hardIron, final Matrix initialMm) {
        return create(groundTruthMagneticFluxDensityNorm, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param measurements                       collection of body magnetic flux density
     *                                           measurements with standard deviation of
     *                                           magnetometer measurements taken at the same
     *                                           position with zero velocity and unknown different
     *                                           orientations.
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(qualityScores, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed) {
        return create(qualityScores, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final double[] hardIron) {
        return create(qualityScores, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix hardIron) {
        return create(qualityScores, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix hardIron, final Matrix initialMm) {
        return create(qualityScores, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        return create(qualityScores, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        return create(qualityScores, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided quality
     *                                  scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        return create(qualityScores, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.*
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        return create(qualityScores, measurements, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, hardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, measurements, commonAxisUsed, hardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param commonAxisUsed                     indicates whether z-axis is assumed to be common
     *                                           for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final boolean commonAxisUsed) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3, or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final double[] hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1, or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final Matrix hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param qualityScores                      quality scores corresponding to each provided
     *                                           measurement. The larger the score value the better
     *                                           the quality of the sample.
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final Matrix hardIron, final Matrix initialMm) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, hardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is
     *                                  smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1,
     *                                  or if provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3, or if provided
     *                                  quality scores length is smaller than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, hardIron, initialMm,
                listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
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
     * @param hardIron                           known hard-iron.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3, or if
     *                                  provided quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibrator create(
            final double[] qualityScores, final Double groundTruthMagneticFluxDensityNorm,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronMagneticFluxDensityNormMagnetometerCalibratorListener listener) {
        return create(qualityScores, groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, hardIron,
                initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Computes error of a preliminary result respect a given measurement.
     *
     * @param measurement       a measurement.
     * @param preliminaryResult a preliminary result.
     * @return computed error.
     */
    protected double computeError(
            final StandardDeviationBodyMagneticFluxDensity measurement, final PreliminaryResult preliminaryResult) {

        try {
            // The magnetometer model is:
            // mBmeas = bm + (I + Mm) * mBtrue

            // mBmeas - bm = (I + Mm) * mBtrue

            // mBtrue = (I + Mm)^-1 * (mBmeas - ba)

            // We know that ||mBtrue||should be equal to the magnitude of the
            // Earth magnetic field at provided location

            final var estMm = preliminaryResult.estimatedMm;

            if (identity == null) {
                identity = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            }

            if (tmp1 == null) {
                tmp1 = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
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

            identity.add(estMm, tmp1);

            Utils.inverse(tmp1, tmp2);

            final var measuredMagneticFluxDensity = measurement.getMagneticFluxDensity();
            final var bMeasX = measuredMagneticFluxDensity.getBx();
            final var bMeasY = measuredMagneticFluxDensity.getBy();
            final var bMeasZ = measuredMagneticFluxDensity.getBz();

            tmp3.setElementAtIndex(0, bMeasX - hardIronX);
            tmp3.setElementAtIndex(1, bMeasY - hardIronY);
            tmp3.setElementAtIndex(2, bMeasZ - hardIronZ);

            tmp2.multiply(tmp3, tmp4);

            final var norm = Utils.normF(tmp4);
            final var diff = groundTruthMagneticFluxDensityNorm - norm;

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

        final var meas = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();

        for (final var samplesIndex : samplesIndices) {
            meas.add(this.measurements.get(samplesIndex));
        }

        try {
            final var result = new PreliminaryResult();
            result.estimatedMm = getInitialMm();

            innerCalibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
            innerCalibrator.setInitialMm(result.estimatedMm);
            innerCalibrator.setCommonAxisUsed(commonAxisUsed);
            innerCalibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
            innerCalibrator.setMeasurements(meas);
            innerCalibrator.calibrate();

            result.estimatedMm = innerCalibrator.getEstimatedMm();

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

            final var inlierMeasurements = new ArrayList<StandardDeviationBodyMagneticFluxDensity>();
            for (var i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(measurements.get(i));
                }
            }

            try {
                innerCalibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);
                innerCalibrator.setInitialMm(preliminaryResult.estimatedMm);
                innerCalibrator.setCommonAxisUsed(commonAxisUsed);
                innerCalibrator.setGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
                innerCalibrator.setMeasurements(inlierMeasurements);
                innerCalibrator.calibrate();

                estimatedMm = innerCalibrator.getEstimatedMm();

                if (keepCovariance) {
                    estimatedCovariance = innerCalibrator.getEstimatedCovariance();
                } else {
                    estimatedCovariance = null;
                }

                estimatedMse = innerCalibrator.getEstimatedMse();
                estimatedChiSq = innerCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                estimatedCovariance = preliminaryResult.covariance;
                estimatedMm = preliminaryResult.estimatedMm;
                estimatedMse = preliminaryResult.estimatedMse;
                estimatedChiSq = preliminaryResult.estimatedChiSq;
            }
        } else {
            estimatedCovariance = preliminaryResult.covariance;
            estimatedMm = preliminaryResult.estimatedMm;
            estimatedMse = preliminaryResult.estimatedMse;
            estimatedChiSq = preliminaryResult.estimatedChiSq;
        }
    }

    /**
     * Internally sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made, expressed in Teslas (T).
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     */
    private void internalSetGroundTruthMagneticFluxDensityNorm(final Double groundTruthMagneticFluxDensityNorm) {
        if (groundTruthMagneticFluxDensityNorm != null && groundTruthMagneticFluxDensityNorm < 0.0) {
            throw new IllegalArgumentException();
        }
        this.groundTruthMagneticFluxDensityNorm = groundTruthMagneticFluxDensityNorm;
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

    /**
     * Internal class containing estimated preliminary result.
     */
    protected static class PreliminaryResult {
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
         * Covariance matrix.
         */
        private Matrix covariance;

        /**
         * Estimated Mean Squared Error (MSE).
         */
        private double estimatedMse;

        /**
         * Estimated chi square value.
         */
        private double estimatedChiSq;
    }
}
