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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.List;

/**
 * This is an abstract class to robustly estimate magnetometer hard-iron
 * biases, cross couplings and scaling factors.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single known
 * position and instant must be taken at 10 different unknown orientations and
 * zero velocity when common z-axis is assumed, otherwise at least 13
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
 * a short span of time where Earth magnetic field can be assumed to be
 * constant at provided location and instant.
 */
public abstract class RobustKnownPositionAndInstantMagnetometerCalibrator implements
        MagnetometerNonLinearCalibrator, UnknownHardIronNonLinearMagnetometerCalibrator,
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
            BaseMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL =
            BaseMagneticFluxDensityNormMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;

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
    protected List<StandardDeviationBodyMagneticFluxDensity> mMeasurements;

    /**
     * Listener to be notified of events such as when calibration starts, ends or its
     * progress significantly changes.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibratorListener mListener;

    /**
     * Indicates whether estimator is running.
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
     * Data related to inlier found after calibration.
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
    protected int mPreliminarySubsetSize = MINIMUM_MEASUREMENTS_GENERAL;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from soft-iron (Mm) matrix.
     */
    private boolean mCommonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] mEstimatedHardIron;

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
    private Matrix mEstimatedMm;

    /**
     * Indicates whether covariance must be kept after refining result.
     * This setting is only taken into account if result is refined.
     */
    private boolean mKeepCovariance = DEFAULT_KEEP_COVARIANCE;

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
     * Initial x-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronX;

    /**
     * Initial y-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronY;

    /**
     * Initial z-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double mInitialHardIronZ;

    /**
     * Initial x scaling factor.
     */
    private double mInitialSx;

    /**
     * Initial y scaling factor.
     */
    private double mInitialSy;

    /**
     * Initial z scaling factor.
     */
    private double mInitialSz;

    /**
     * Initial x-y cross coupling error.
     */
    private double mInitialMxy;

    /**
     * Initial x-z cross coupling error.
     */
    private double mInitialMxz;

    /**
     * Initial y-x cross coupling error.
     */
    private double mInitialMyx;

    /**
     * Initial y-z cross coupling error.
     */
    private double mInitialMyz;

    /**
     * Initial z-x cross coupling error.
     */
    private double mInitialMzx;

    /**
     * Initial z-y cross coupling error.
     */
    private double mInitialMzy;

    /**
     * Position where body magnetic flux density measurements have been
     * taken.
     */
    private NEDPosition mPosition;

    /**
     * Timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     */
    private Double mYear = convertTime(System.currentTimeMillis());

    /**
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel mMagneticModel;

    /**
     * Inner calibrator to compute calibration for each subset of data or during
     * final refining.
     */
    private final KnownPositionAndInstantMagnetometerCalibrator mInnerCalibrator =
            new KnownPositionAndInstantMagnetometerCalibrator();

    /**
     * Contains magnetic field norm for current position to be reused
     * during calibration.
     */
    protected double mMagneticDensityNorm;

    /**
     * Contains 3x3 identify to be reused.
     */
    protected Matrix mIdentity;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix mTmp1;

    /**
     * Contains 3x3 temporary matrix.
     */
    protected Matrix mTmp2;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix mTmp3;

    /**
     * Contains 3x1 temporary matrix.
     */
    protected Matrix mTmp4;

    /**
     * Constructor.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        mListener = listener;
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final boolean commonAxisUsed) {
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final WorldMagneticModel magneticModel) {
        mMagneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final double[] initialHardIron) {
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final Matrix initialHardIron) {
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron);
        try {
            setInitialMm(initialMm);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final NEDPosition position) {
        mPosition = position;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(position);
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(position, measurements);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(initialHardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(position, measurements, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(initialHardIron);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(position, measurements, initialHardIron);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        mPosition = position;
        mMeasurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        this(position, measurements, initialHardIron, initialMm);
        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(position, measurements, commonAxisUsed, initialHardIron, initialMm);
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(final ECEFPosition position) {
        this(convertPosition(position));
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this(convertPosition(position), measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(convertPosition(position), measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(convertPosition(position), measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(convertPosition(position), measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, initialHardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
    protected RobustKnownPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        this(convertPosition(position), measurements, commonAxisUsed, initialHardIron, initialMm, listener);
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made,
     * expressed in Teslas (T).
     *
     * @return ground truth magnetic flux density or null.
     */
    public Double getGroundTruthMagneticFluxDensityNorm() {
        return mInnerCalibrator.getGroundTruthMagneticFluxDensityNorm();
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made.
     *
     * @return ground truth magnetic flux density or null.
     */
    public MagneticFluxDensity getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity() {
        return mInnerCalibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity();
    }

    /**
     * Gets ground truth magnetic flux density norm to be expected at location where measurements have been made.
     *
     * @param result instance where result will be stored.
     * @return true if ground truth magnetic flux density norm has been defined, false if it is not available yet.
     */
    public boolean getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(final MagneticFluxDensity result) {
        return mInnerCalibrator.getGroundTruthMagneticFluxDensityNormAsMagneticFluxDensity(result);
    }

    /**
     * Gets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronX() {
        return mInitialHardIronX;
    }

    /**
     * Sets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronX(final double initialHardIronX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = initialHardIronX;
    }

    /**
     * Gets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronY() {
        return mInitialHardIronY;
    }

    /**
     * Sets initial y-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronY(final double initialHardIronY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronY = initialHardIronY;
    }

    /**
     * Gets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronZ() {
        return mInitialHardIronZ;
    }

    /**
     * Sets initial z-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in meters Teslas (T).
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronZ(final double initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronZ = initialHardIronZ;
    }

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronX, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronX);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronX(final MagneticFluxDensity initialHardIronX) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = convertMagneticFluxDensity(initialHardIronX);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronY, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronY);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronY(final MagneticFluxDensity initialHardIronY) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronY = convertMagneticFluxDensity(initialHardIronY);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(mInitialHardIronZ, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(mInitialHardIronZ);
        result.setUnit(MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIronZ(final MagneticFluxDensity initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
    }

    /**
     * Sets initial hard-iron bias coordinates of magnetometer used to find
     * a solution expressed in Teslas (T).
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ)
            throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mInitialHardIronX = initialHardIronX;
        mInitialHardIronY = initialHardIronY;
        mInitialHardIronZ = initialHardIronZ;
    }

    /**
     * Sets initial hard iron coordinates of magnetometer used to find a solution.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer bias.
     * @param initialHardIronY initial y-coordinate of magnetometer bias.
     * @param initialHardIronZ initial z-coordinate of magnetometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(
            final MagneticFluxDensity initialHardIronX, final MagneticFluxDensity initialHardIronY,
            final MagneticFluxDensity initialHardIronZ) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitialHardIronX = convertMagneticFluxDensity(initialHardIronX);
        mInitialHardIronY = convertMagneticFluxDensity(initialHardIronY);
        mInitialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @return initial hard-iron.
     */
    @Override
    public MagneticFluxDensityTriad getInitialHardIronAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                mInitialHardIronX, mInitialHardIronY, mInitialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(mInitialHardIronX, mInitialHardIronY, mInitialHardIronZ,
                MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Sets initial hard-iron used to find a solution.
     *
     * @param initialHardIron initial hard-iron to be set.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialHardIron(final MagneticFluxDensityTriad initialHardIron) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mInitialHardIronX = convertMagneticFluxDensity(initialHardIron.getValueX(), initialHardIron.getUnit());
        mInitialHardIronY = convertMagneticFluxDensity(initialHardIron.getValueY(), initialHardIron.getUnit());
        mInitialHardIronZ = convertMagneticFluxDensity(initialHardIron.getValueZ(), initialHardIron.getUnit());
    }

    /**
     * Gets initial x scaling factor.
     *
     * @return initial x scaling factor.
     */
    @Override
    public double getInitialSx() {
        return mInitialSx;
    }

    /**
     * Sets initial x scaling factor.
     *
     * @param initialSx initial x scaling factor.
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
     * Gets initial y scaling factor.
     *
     * @return initial y scaling factor.
     */
    @Override
    public double getInitialSy() {
        return mInitialSy;
    }

    /**
     * Sets initial y scaling factor.
     *
     * @param initialSy initial y scaling factor.
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
     * Gets initial z scaling factor.
     *
     * @return initial z scaling factor.
     */
    @Override
    public double getInitialSz() {
        return mInitialSz;
    }

    /**
     * Sets initial z scaling factor.
     *
     * @param initialSz initial z scaling factor.
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
     * Gets initial x-y cross coupling error.
     *
     * @return initial x-y cross coupling error.
     */
    @Override
    public double getInitialMxy() {
        return mInitialMxy;
    }

    /**
     * Sets initial x-y cross coupling error.
     *
     * @param initialMxy initial x-y cross coupling error.
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
     * Gets initial x-z cross coupling error.
     *
     * @return initial x-z cross coupling error.
     */
    @Override
    public double getInitialMxz() {
        return mInitialMxz;
    }

    /**
     * Sets initial x-z cross coupling error.
     *
     * @param initialMxz initial x-z cross coupling error.
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
     * Gets initial y-x cross coupling error.
     *
     * @return initial y-x cross coupling error.
     */
    @Override
    public double getInitialMyx() {
        return mInitialMyx;
    }

    /**
     * Sets initial y-x cross coupling error.
     *
     * @param initialMyx initial y-x cross coupling error.
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
     * Gets initial y-z cross coupling error.
     *
     * @return initial y-z cross coupling error.
     */
    @Override
    public double getInitialMyz() {
        return mInitialMyz;
    }

    /**
     * Sets initial y-z cross coupling error.
     *
     * @param initialMyz initial y-z cross coupling error.
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
     * Gets initial z-x cross coupling error.
     *
     * @return initial z-x cross coupling error.
     */
    @Override
    public double getInitialMzx() {
        return mInitialMzx;
    }

    /**
     * Sets initial z-x cross coupling error.
     *
     * @param initialMzx initial z-x cross coupling error.
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
     * Gets initial z-y cross coupling error.
     *
     * @return initial z-y cross coupling error.
     */
    @Override
    public double getInitialMzy() {
        return mInitialMzy;
    }

    /**
     * Sets initial z-y cross coupling error.
     *
     * @param initialMzy initial z-y cross coupling error.
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
        if (mRunning) {
            throw new LockedException();
        }
        mInitialSx = initialSx;
        mInitialSy = initialSy;
        mInitialSz = initialSz;
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
        if (mRunning) {
            throw new LockedException();
        }
        setInitialScalingFactors(initialSx, initialSy, initialSz);
        setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getInitialHardIron() {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        getInitialHardIron(result);
        return result;
    }

    /**
     * Gets initial hard-iron  bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have
     *                                  length 3.
     */
    @Override
    public void getInitialHardIron(final double[] result) {
        if (result.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = mInitialHardIronX;
        result[1] = mInitialHardIronY;
        result[2] = mInitialHardIronZ;
    }

    /**
     * Sets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    @Override
    public void setInitialHardIron(final double[] initialHardIron) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        if (initialHardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        mInitialHardIronX = initialHardIron[0];
        mInitialHardIronY = initialHardIron[1];
        mInitialHardIronZ = initialHardIron[2];
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     * Values are expressed in Teslas (T).
     *
     * @return initial hard-iron bias to be used to find a solution as a
     * column matrix.
     */
    @Override
    public Matrix getInitialHardIronAsMatrix() {
        Matrix result;
        try {
            result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
            getInitialHardIronAsMatrix(result);
        } catch (final WrongSizeException ignore) {
            // never happens
            result = null;
        }
        return result;
    }

    /**
     * Gets initial hard-iron bias to be used to find a solution as a
     * column matrix.
     * Values are expressed in Teslas (T).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void getInitialHardIronAsMatrix(final Matrix result) {
        if (result.getRows() != BodyMagneticFluxDensity.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, mInitialHardIronX);
        result.setElementAtIndex(1, mInitialHardIronY);
        result.setElementAtIndex(2, mInitialHardIronZ);
    }

    /**
     * Sets initial hard-iron bias to be used to find a solution as a column
     * matrix with values expressed in Teslas (T).
     *
     * @param initialHardIron initial hard-iron bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setInitialHardIron(final Matrix initialHardIron) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialHardIron.getRows() != BodyMagneticFluxDensity.COMPONENTS || initialHardIron.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        mInitialHardIronX = initialHardIron.getElementAtIndex(0);
        mInitialHardIronY = initialHardIron.getElementAtIndex(1);
        mInitialHardIronZ = initialHardIron.getElementAtIndex(2);
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
     * Sets initial scale factors and cross coupling errors matrix.
     *
     * @param initialMm initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setInitialMm(final Matrix initialMm) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        if (initialMm.getRows() != BodyKinematics.COMPONENTS || initialMm.getColumns() != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }

        mInitialSx = initialMm.getElementAtIndex(0);
        mInitialMyx = initialMm.getElementAtIndex(1);
        mInitialMzx = initialMm.getElementAtIndex(2);

        mInitialMxy = initialMm.getElementAtIndex(3);
        mInitialSy = initialMm.getElementAtIndex(4);
        mInitialMzy = initialMm.getElementAtIndex(5);

        mInitialMxz = initialMm.getElementAtIndex(6);
        mInitialMyz = initialMm.getElementAtIndex(7);
        mInitialSz = initialMm.getElementAtIndex(8);
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken.
     *
     * @return position where body magnetic flux density measurements have
     * been taken.
     */
    public NEDPosition getNedPosition() {
        return mPosition;
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final NEDPosition position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = position;
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @return position where body magnetic flux density measurements have
     * been taken or null if not available.
     */
    public ECEFPosition getEcefPosition() {
        if (mPosition != null) {
            final ECEFPosition result = new ECEFPosition();
            getEcefPosition(result);
            return result;
        } else {
            return null;
        }
    }

    /**
     * Gets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if ECEF position could be computed, false otherwise.
     */
    public boolean getEcefPosition(final ECEFPosition result) {

        if (mPosition != null) {
            final ECEFVelocity velocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    mPosition.getLatitude(), mPosition.getLongitude(), mPosition.getHeight(),
                    0.0, 0.0, 0.0, result, velocity);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets position where body magnetic flux density measurements have been
     * taken expressed in ECEF coordinates.
     *
     * @param position position where body magnetic flux density have been
     *                 taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final ECEFPosition position) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mPosition = convertPosition(position);
    }

    /**
     * Gets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @return timestamp expressed as decimal year or null if not defined.
     */
    public Double getYear() {
        return mYear;
    }

    /**
     * Sets timestamp expressed as decimal year where magnetic flux density
     * measurements have been measured.
     *
     * @param year timestamp expressed as decimal year.
     * @throws LockedException if calibrator is currently running.
     */
    public void setYear(final Double year) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = year;
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param timestampMillis a timestamp expressed in milliseconds since
     *                        epoch time (January 1st, 1970 at midnight).
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Long timestampMillis) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(timestampMillis);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param date a date instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final Date date) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(date);
    }

    /**
     * Sets timestamp when magnetic flux density measurements have been
     * measured.
     *
     * @param calendar a calendar instance containing a timestamp.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTime(final GregorianCalendar calendar) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mYear = convertTime(calendar);
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
        return mMeasurements;
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
    public void setMeasurements(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMeasurements = measurements;
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
        return mCommonAxisUsed;
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
        if (mRunning) {
            throw new LockedException();
        }

        mCommonAxisUsed = commonAxisUsed;
    }

    /**
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    public RobustKnownPositionAndInstantMagnetometerCalibratorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }

        mListener = listener;
    }

    /**
     * Gets minimum number of required measurements.
     *
     * @return minimum number of required measurements.
     */
    @Override
    public int getMinimumRequiredMeasurements() {
        return mCommonAxisUsed ? MINIMUM_MEASUREMENTS_COMMON_Z_AXIS : MINIMUM_MEASUREMENTS_GENERAL;
    }

    /**
     * Indicates whether calibrator is ready to start the estimator.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mMeasurements != null && mMeasurements.size() >= getMinimumRequiredMeasurements()
                && mPosition != null && mYear != null;
    }

    /**
     * Indicates whether calibrator is currently running or no.
     *
     * @return true if calibrator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return mRunning;
    }

    /**
     * Gets Earth's magnetic model.
     *
     * @return Earth's magnetic model or null if not provided.
     */
    public WorldMagneticModel getMagneticModel() {
        return mMagneticModel;
    }

    /**
     * Sets Earth's magnetic model.
     *
     * @param magneticModel Earth's magnetic model to be set.
     * @throws LockedException if calibrator is currently running.
     */
    public void setMagneticModel(final WorldMagneticModel magneticModel) throws LockedException {
        if (mRunning) {
            throw new LockedException();
        }
        mMagneticModel = magneticModel;
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
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    @Override
    public double[] getEstimatedHardIron() {
        return mEstimatedHardIron;
    }

    /**
     * Gets array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where estimated magnetometer biases will be
     *               stored.
     * @return true if result instance was updated, false otherwise (when
     * estimation is not yet available).
     */
    @Override
    public boolean getEstimatedHardIron(final double[] result) {
        if (mEstimatedHardIron != null) {
            System.arraycopy(mEstimatedHardIron, 0, result, 0, mEstimatedHardIron.length);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @return column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases.
     */
    @Override
    public Matrix getEstimatedHardIronAsMatrix() {
        return mEstimatedHardIron != null ? Matrix.newFromArray(mEstimatedHardIron) : null;
    }

    /**
     * Gets column matrix containing x,y,z components of estimated
     * magnetometer hard-iron biases expressed in Teslas (T).
     *
     * @param result instance where result data will be stored.
     * @return true if result was updated, false otherwise.
     * @throws WrongSizeException if provided result instance has invalid size.
     */
    @Override
    public boolean getEstimatedHardIronAsMatrix(final Matrix result) throws WrongSizeException {
        if (mEstimatedHardIron != null) {
            result.fromArray(mEstimatedHardIron);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets x coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return x coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronX() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[0] : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return y coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronY() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[1] : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias expressed in
     * Teslas (T).
     *
     * @return z coordinate of estimated magnetometer bias or null if not
     * available.
     */
    @Override
    public Double getEstimatedHardIronZ() {
        return mEstimatedHardIron != null ? mEstimatedHardIron[2] : null;
    }

    /**
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @return x coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronXAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[0], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[0]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets y coordinate of estimated magnetometer bias.
     *
     * @return y coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronYAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[1], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[1]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets z coordinate of estimated magnetometer bias.
     *
     * @return z coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronZAsMagneticFluxDensity() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensity(mEstimatedHardIron[2], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedHardIron != null) {
            result.setValue(mEstimatedHardIron[2]);
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets estimated magnetometer bias.
     *
     * @return estimated magnetometer bias or null if not available.
     */
    @Override
    public MagneticFluxDensityTriad getEstimatedHardIronAsTriad() {
        return mEstimatedHardIron != null ?
                new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                        mEstimatedHardIron[0], mEstimatedHardIron[1], mEstimatedHardIron[2]) : null;
    }

    /**
     * Gets estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available and result was
     * modified, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronAsTriad(final MagneticFluxDensityTriad result) {
        if (mEstimatedHardIron != null) {
            result.setValueCoordinatesAndUnit(mEstimatedHardIron[0], mEstimatedHardIron[1], mEstimatedHardIron[2],
                    MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
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
        return mEstimatedMm;
    }

    /**
     * Gets estimated x-axis scale factor.
     *
     * @return estimated x-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSx() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y-axis scale factor.
     *
     * @return estimated y-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSy() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z-axis scale factor.
     *
     * @return estimated z-axis scale factor or null if not available.
     */
    @Override
    public Double getEstimatedSz() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(2, 2) : null;
    }

    /**
     * Gets estimated x-y cross-coupling error.
     *
     * @return estimated x-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxy() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(0, 1) : null;
    }

    /**
     * Gets estimated x-z cross-coupling error.
     *
     * @return estimated x-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMxz() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(0, 2) : null;
    }

    /**
     * Gets estimated y-x cross-coupling error.
     *
     * @return estimated y-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyx() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(1, 0) : null;
    }

    /**
     * Gets estimated y-z cross-coupling error.
     *
     * @return estimated y-z cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMyz() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(1, 2) : null;
    }

    /**
     * Gets estimated z-x cross-coupling error.
     *
     * @return estimated z-x cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzx() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(2, 0) : null;
    }

    /**
     * Gets estimated z-y cross-coupling error.
     *
     * @return estimated z-y cross-coupling error or null if not available.
     */
    @Override
    public Double getEstimatedMzy() {
        return mEstimatedMm != null ? mEstimatedMm.getElementAt(2, 1) : null;
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
     * Gets estimated covariance matrix for estimated calibration parameters.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): bx, by, bz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy.
     *
     * @return estimated covariance matrix for estimated calibration parameters.
     */
    @Override
    public Matrix getEstimatedCovariance() {
        return mEstimatedCovariance;
    }

    /**
     * Gets variance of estimated x coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated x coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronXVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronXStandardDeviation() {
        final Double variance = getEstimatedHardIronXVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronXStandardDeviation(), MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronXStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated y coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated y coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronYVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronYStandardDeviation() {
        final Double variance = getEstimatedHardIronYVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronYStandardDeviation(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronYStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets variance of estimated z coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated z coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronZVariance() {
        return mEstimatedCovariance != null ? mEstimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronZStandardDeviation() {
        final Double variance = getEstimatedHardIronZVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronZStandardDeviation(), MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronZStandardDeviation());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets standard deviation of estimated magnetometer bias coordinates.
     *
     * @return standard deviation of estimated magnetometer bias coordinates.
     */
    public MagneticFluxDensityTriad getEstimatedHardIronStandardDeviation() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                        getEstimatedHardIronXStandardDeviation(),
                        getEstimatedHardIronYStandardDeviation(),
                        getEstimatedHardIronZStandardDeviation()) : null;
    }

    /**
     * Gets standard deviation of estimated magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of magnetometer bias was available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviation(final MagneticFluxDensityTriad result) {
        if (mEstimatedCovariance != null) {
            result.setValueCoordinatesAndUnit(
                    getEstimatedHardIronXStandardDeviation(),
                    getEstimatedHardIronYStandardDeviation(),
                    getEstimatedHardIronZStandardDeviation(),
                    MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates
     * expressed in Teslas (T).
     *
     * @return average of estimated standard deviation of magnetometer bias coordinates,
     * or null if not available.
     */
    public Double getEstimatedHardIronStandardDeviationAverage() {
        return mEstimatedCovariance != null ?
                (getEstimatedHardIronXStandardDeviation() + getEstimatedHardIronYStandardDeviation() +
                        getEstimatedHardIronZStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @return average of estimated standard deviation of magnetometer bias coordinates,
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronStandardDeviationAverage(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of magnetometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationAverage());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias expressed in
     * Teslas (T).
     *
     * @return norm of estimated standard deviation of magnetometer bias or null
     * if not available.
     */
    public Double getEstimatedHardIronStandardDeviationNorm() {
        return mEstimatedCovariance != null ?
                Math.sqrt(getEstimatedHardIronXVariance() + getEstimatedHardIronYVariance()
                        + getEstimatedHardIronZVariance()) : null;
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias.
     *
     * @return norm of estimated standard deviation of magnetometer bias or null
     * if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity() {
        return mEstimatedCovariance != null ?
                new MagneticFluxDensity(getEstimatedHardIronStandardDeviationNorm(),
                        MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if norm of estimated standard deviation of magnetometer bias
     * is available, false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (mEstimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationNorm());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets size of subsets to be checked during robust estimation.
     * This has to be at least {@link #MINIMUM_MEASUREMENTS_COMMON_Z_AXIS}.
     *
     * @return size of subsets to be checked during robust estimation.
     */
    public int getPreliminarySubsetSize() {
        return mPreliminarySubsetSize;
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
        if (mRunning) {
            throw new LockedException();
        }
        if (preliminarySubsetSize < MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
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
     * Creates a robust magnetometer calibrator.
     *
     * @param method robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator();
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator();
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator();
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator();
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator();
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(listener);
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final WorldMagneticModel magneticModel, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final Matrix initialHardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @param method   robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @param method       robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, measurements);
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
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final boolean commonAxisUsed, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final WorldMagneticModel magneticModel, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(magneticModel);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, magneticModel);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, magneticModel);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final Matrix initialHardIron, final Matrix initialMm,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, position);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, position);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @param method        robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @param method         robust estimator method
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, position);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(qualityScores, position);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param method        robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(position, measurements);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm, final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm);
        };
    }

    /**
     * Creates a robust magnetometer calibrator.
     *
     * @param qualityScores   quality scores corresponding to each provided
     *                        measurement. The larger the score value the better
     *                        the quality of the sample.
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @param method          robust estimator method.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 10 samples.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener,
            final RobustEstimatorMethod method) {
        return switch (method) {
            case RANSAC -> new RANSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case LMEDS -> new LMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case MSAC -> new MSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            case PROSAC -> new PROSACRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
            default -> new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator(
                    qualityScores, position, measurements, commonAxisUsed, initialHardIron, initialMm, listener);
        };
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final boolean commonAxisUsed) {
        return create(commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final WorldMagneticModel magneticModel) {
        return create(magneticModel, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final double[] initialHardIron) {
        return create(initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final Matrix initialHardIron) {
        return create(initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final Matrix initialHardIron, final Matrix initialMm) {
        return create(initialHardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final NEDPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        return create(position, measurements, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        return create(position, measurements, commonAxisUsed, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        return create(position, measurements, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        return create(position, measurements, commonAxisUsed, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        return create(position, measurements, initialHardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        return create(position, measurements, commonAxisUsed, initialHardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(final ECEFPosition position) {
        return create(position, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        return create(position, measurements, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @return a robust magnetometer calibrator.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        return create(position, measurements, commonAxisUsed, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
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
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        return create(position, measurements, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        return create(position, measurements, commonAxisUsed, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        return create(position, measurements, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        return create(position, measurements, commonAxisUsed, initialHardIron, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        return create(position, measurements, initialHardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, initialHardIron, initialMm, listener, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        return create(position, measurements, commonAxisUsed, initialHardIron, initialMm, DEFAULT_ROBUST_METHOD);
    }

    /**
     * Creates a robust magnetometer calibrator with default robust method.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
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
     * @return a robust magnetometer calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public static RobustKnownPositionAndInstantMagnetometerCalibrator create(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final RobustKnownPositionAndInstantMagnetometerCalibratorListener listener) {
        return create(position, measurements, commonAxisUsed, initialHardIron, initialMm, listener,
                DEFAULT_ROBUST_METHOD);
    }

    /**
     * Initializes calibrator and estimates magnetic flux density norm
     * at provided position and timestamp.
     *
     * @throws IOException if world magnetic model cannot be loaded.
     */
    protected void initialize() throws IOException {
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (mMagneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(mMagneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final NEDPosition position = getNedPosition();
        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(position, mYear);
        mMagneticDensityNorm = earthB.getNorm();
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

            final double[] estimatedBiases = preliminaryResult.mEstimatedHardIron;
            final Matrix estimatedMm = preliminaryResult.mEstimatedMm;

            if (mIdentity == null) {
                mIdentity = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            }

            if (mTmp1 == null) {
                mTmp1 = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            }

            if (mTmp2 == null) {
                mTmp2 = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }

            if (mTmp3 == null) {
                mTmp3 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            if (mTmp4 == null) {
                mTmp4 = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            mIdentity.add(estimatedMm, mTmp1);

            Utils.inverse(mTmp1, mTmp2);

            final BodyMagneticFluxDensity measuredMagneticFluxDensity = measurement.getMagneticFluxDensity();
            final double bMeasX = measuredMagneticFluxDensity.getBx();
            final double bMeasY = measuredMagneticFluxDensity.getBy();
            final double bMeasZ = measuredMagneticFluxDensity.getBz();

            final double bx = estimatedBiases[0];
            final double by = estimatedBiases[1];
            final double bz = estimatedBiases[2];

            mTmp3.setElementAtIndex(0, bMeasX - bx);
            mTmp3.setElementAtIndex(1, bMeasY - by);
            mTmp3.setElementAtIndex(2, bMeasZ - bz);

            mTmp2.multiply(mTmp3, mTmp4);

            final double norm = Utils.normF(mTmp4);
            final double diff = mMagneticDensityNorm - norm;

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

        final List<StandardDeviationBodyMagneticFluxDensity> measurements = new ArrayList<>();

        for (final int samplesIndex : samplesIndices) {
            measurements.add(mMeasurements.get(samplesIndex));
        }

        try {
            final PreliminaryResult result = new PreliminaryResult();
            result.mEstimatedHardIron = getInitialHardIron();
            result.mEstimatedMm = getInitialMm();

            mInnerCalibrator.setInitialHardIron(result.mEstimatedHardIron);
            mInnerCalibrator.setInitialMm(result.mEstimatedMm);
            mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
            mInnerCalibrator.setPosition(mPosition);
            mInnerCalibrator.setYear(mYear);
            mInnerCalibrator.setMeasurements(measurements);
            mInnerCalibrator.calibrate();

            mInnerCalibrator.getEstimatedHardIron(result.mEstimatedHardIron);
            result.mEstimatedMm = mInnerCalibrator.getEstimatedMm();

            if (mKeepCovariance) {
                result.mCovariance = mInnerCalibrator.getEstimatedCovariance();
            } else {
                result.mCovariance = null;
            }

            result.mEstimatedMse = mInnerCalibrator.getEstimatedMse();
            result.mEstimatedChiSq = mInnerCalibrator.getEstimatedHardIronX();

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
            final int nSamples = mMeasurements.size();

            final List<StandardDeviationBodyMagneticFluxDensity> inlierMeasurements = new ArrayList<>();
            for (int i = 0; i < nSamples; i++) {
                if (inliers.get(i)) {
                    // sample is inlier
                    inlierMeasurements.add(mMeasurements.get(i));
                }
            }

            try {
                mInnerCalibrator.setInitialHardIron(preliminaryResult.mEstimatedHardIron);
                mInnerCalibrator.setInitialMm(preliminaryResult.mEstimatedMm);
                mInnerCalibrator.setCommonAxisUsed(mCommonAxisUsed);
                mInnerCalibrator.setPosition(mPosition);
                mInnerCalibrator.setYear(mYear);
                mInnerCalibrator.setMeasurements(inlierMeasurements);
                mInnerCalibrator.calibrate();

                mEstimatedHardIron = mInnerCalibrator.getEstimatedHardIron();
                mEstimatedMm = mInnerCalibrator.getEstimatedMm();

                if (mKeepCovariance) {
                    mEstimatedCovariance = mInnerCalibrator.getEstimatedCovariance();
                } else {
                    mEstimatedCovariance = null;
                }

                mEstimatedMse = mInnerCalibrator.getEstimatedMse();
                mEstimatedChiSq = mInnerCalibrator.getEstimatedChiSq();

            } catch (final LockedException | CalibrationException | NotReadyException e) {
                mEstimatedCovariance = preliminaryResult.mCovariance;
                mEstimatedHardIron = preliminaryResult.mEstimatedHardIron;
                mEstimatedMm = preliminaryResult.mEstimatedMm;
                mEstimatedMse = preliminaryResult.mEstimatedMse;
                mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
            }
        } else {
            mEstimatedCovariance = preliminaryResult.mCovariance;
            mEstimatedHardIron = preliminaryResult.mEstimatedHardIron;
            mEstimatedMm = preliminaryResult.mEstimatedMm;
            mEstimatedMse = preliminaryResult.mEstimatedMse;
            mEstimatedChiSq = preliminaryResult.mEstimatedChiSq;
        }
    }

    /**
     * Converts a time instance expressed in milliseconds since epoch time
     * (January 1st, 1970 at midnight) to a decimal year.
     *
     * @param timestampMillis milliseconds value to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Long timestampMillis) {
        if (timestampMillis == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestampMillis);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained ina date object to a
     * decimal year.
     *
     * @param date a time instance to be converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final Date date) {
        if (date == null) {
            return null;
        }

        final GregorianCalendar calendar = new GregorianCalendar();
        calendar.setTime(date);
        return convertTime(calendar);
    }

    /**
     * Converts a time instant contained in a gregorian calendar to a
     * decimal year.
     *
     * @param calendar calendar containing a specific instant to be
     *                 converted.
     * @return converted value expressed in decimal years.
     */
    private static Double convertTime(final GregorianCalendar calendar) {
        if (calendar == null) {
            return null;
        }

        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }

    /**
     * Converts provided ECEF position to position expressed in NED
     * coordinates.
     *
     * @param position ECEF position to be converted.
     * @return converted position expressed in NED coordinates.
     */
    private static NEDPosition convertPosition(final ECEFPosition position) {
        final NEDVelocity velocity = new NEDVelocity();
        final NEDPosition result = new NEDPosition();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                position.getX(), position.getY(), position.getZ(),
                0.0, 0.0, 0.0, result, velocity);
        return result;
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
         * Estimated magnetometer hard-iron biases for each magnetometer axis
         * expressed in Teslas (T).
         */
        private double[] mEstimatedHardIron;

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
        private Matrix mEstimatedMm;

        /**
         * Covariance matrix.
         */
        private Matrix mCovariance;

        /**
         * Estimated Mean Squared Error (MSE).
         */
        private double mEstimatedMse;

        /**
         * Estimated chi square value.
         */
        private double mEstimatedChiSq;
    }
}
