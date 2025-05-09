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
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.util.Collection;

/**
 * Abstract class to estimate magnetometer hard-iron biases, cross couplings and scaling factors.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared
 * error solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single unknown position and unknown orientations
 * when common z-axis is assumed, otherwise at least 13 measurements are required.
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
 * Notice that this calibrator assumes that all measurements are taken in a short span of time,
 * where Earth magnetic field can be assumed to be constant.
 *
 * @param <C> Calibrator type.
 * @param <L> Listener type.
 */
public abstract class BaseMagneticFluxDensityNormMagnetometerCalibrator<
        C extends BaseMagneticFluxDensityNormMagnetometerCalibrator<?, ?>,
        L extends BaseMagneticFluxDensityNormMagnetometerCalibratorListener<C>> implements
        MagnetometerNonLinearCalibrator, UnknownHardIronNonLinearMagnetometerCalibrator,
        UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Number of unknowns when common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 12;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS = COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL = GENERAL_UNKNOWNS + 1;

    /**
     * Ground truth magnetic flux density norm to be expected at location where measurements have been made,
     * expressed in Teslas (T).
     */
    protected Double groundTruthMagneticFluxDensityNorm;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter fitter = new LevenbergMarquardtMultiDimensionFitter();

    /**
     * Initial x-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double initialHardIronX;

    /**
     * Initial y-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double initialHardIronY;

    /**
     * Initial z-coordinate of hard-iron bias to be used to find a solution.
     * This is expressed in Teslas (T).
     */
    private double initialHardIronZ;

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
     * Contains a collection of body magnetic flux density measurements taken
     * at a given position with different unknown orientations and containing the
     * standard deviation of magnetometer measurements.
     */
    private Collection<StandardDeviationBodyMagneticFluxDensity> measurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private L listener;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] estimatedHardIron;

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
    private boolean running;

    /**
     * Internally holds x-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double bmeasX;

    /**
     * Internally holds y-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double bmeasY;

    /**
     * Internally holds z-coordinate of measured magnetic flux density
     * during calibration.
     */
    private double bmeasZ;

    /**
     * Internally holds measured magnetic flux density during calibration
     * expressed as a column matrix.
     */
    private Matrix bmeas;

    /**
     * Internally holds cross-coupling errors during calibration.
     */
    private Matrix m;

    /**
     * Internally holds inverse of cross-coupling errors during calibration.
     */
    private Matrix invM;

    /**
     * Internally holds biases during calibration.
     */
    private Matrix b;

    /**
     * Internally holds computed true magnetic flux density during
     * calibration.
     */
    private Matrix btrue;

    /**
     * Constructor.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final L listener) {
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
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final double[] initialHardIron) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final Matrix initialHardIron) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final Matrix initialHardIron, final Matrix initialMm) {
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
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final L listener) {
        this(measurements);
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        this(measurements);
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final L listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
        this(initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final L listener) {
        this(measurements, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron) {
        this(measurements, initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final L listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
        this(initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final L listener) {
        this(measurements, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        this(measurements, initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final L listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm) {
        this(initialHardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final L listener) {
        this(measurements, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(measurements, initialHardIron, initialMm);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm, final L listener) {
        this(measurements, commonAxisUsed, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(final Double groundTruthMagneticFluxDensityNorm) {
        internalSetGroundTruthMagneticFluxDensityNorm(groundTruthMagneticFluxDensityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final L listener) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final boolean commonAxisUsed) {
        this(groundTruthMagneticFluxDensityNorm);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron array does
     *                                  not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final double[] initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm);
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm);
        try {
            setInitialHardIron(initialHardIron);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm expressed in Teslas (T).
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is
     *                                  negative, or if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm, final Matrix initialHardIron, final Matrix initialMm) {
        this(groundTruthMagneticFluxDensityNorm, initialHardIron);
        try {
            setInitialMm(initialMm);
        } catch (final LockedException ignore) {
            // never happens
        }
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final L listener) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
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
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final L listener) {
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm, initialHardIron);
        this.measurements = measurements;
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] initialHardIron,
            final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron array does not have length 3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] initialHardIron, final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm, initialHardIron);
        this.measurements = measurements;
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm) {
        this(groundTruthMagneticFluxDensityNorm, initialHardIron, initialMm);
        this.measurements = measurements;
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1 or if
     *                                  soft-iron matrix is not 3x3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final Matrix initialMm, final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(groundTruthMagneticFluxDensityNorm, measurements, initialHardIron, initialMm);
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
     * @param initialHardIron                    initial hard-iron to find a solution.
     * @param initialMm                          initial soft-iron matrix containing scale factors
     *                                           and cross coupling errors.
     * @param listener                           listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided magnetic flux norm value is negative,
     *                                  or if provided hard-iron matrix is not 3x1
     *                                  or if soft-iron matrix is not 3x3.
     */
    protected BaseMagneticFluxDensityNormMagnetometerCalibrator(
            final Double groundTruthMagneticFluxDensityNorm,
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm, final L listener) {
        this(groundTruthMagneticFluxDensityNorm, measurements, commonAxisUsed, initialHardIron, initialMm);
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
     * Gets initial x-coordinate of magnetometer hard-iron bias to be used
     * to find a solution.
     * This is expressed in Teslas (T).
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public double getInitialHardIronX() {
        return initialHardIronX;
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronX = initialHardIronX;
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
        return initialHardIronY;
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronY = initialHardIronY;
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
        return initialHardIronZ;
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronZ = initialHardIronZ;
    }

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial x-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronXAsMagneticFluxDensity() {
        return new MagneticFluxDensity(initialHardIronX, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial x-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(initialHardIronX);
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronX = convertMagneticFluxDensity(initialHardIronX);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial y-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronYAsMagneticFluxDensity() {
        return new MagneticFluxDensity(initialHardIronY, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial y-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(initialHardIronY);
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronY = convertMagneticFluxDensity(initialHardIronY);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @return initial z-coordinate of magnetometer hard-iron bias.
     */
    @Override
    public MagneticFluxDensity getInitialHardIronZAsMagneticFluxDensity() {
        return new MagneticFluxDensity(initialHardIronZ, MagneticFluxDensityUnit.TESLA);
    }

    /**
     * Gets initial z-coordinate of magnetometer hard iron bias to be used
     * to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        result.setValue(initialHardIronZ);
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
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
        if (running) {
            throw new LockedException();
        }
        this.initialHardIronX = initialHardIronX;
        this.initialHardIronY = initialHardIronY;
        this.initialHardIronZ = initialHardIronZ;
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
        if (running) {
            throw new LockedException();
        }

        this.initialHardIronX = convertMagneticFluxDensity(initialHardIronX);
        this.initialHardIronY = convertMagneticFluxDensity(initialHardIronY);
        this.initialHardIronZ = convertMagneticFluxDensity(initialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @return initial hard-iron.
     */
    @Override
    public MagneticFluxDensityTriad getInitialHardIronAsTriad() {
        return new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                initialHardIronX, initialHardIronY, initialHardIronZ);
    }

    /**
     * Gets initial hard-iron used to find a solution.
     *
     * @param result instance where result will be stored.
     */
    @Override
    public void getInitialHardIronAsTriad(final MagneticFluxDensityTriad result) {
        result.setValueCoordinatesAndUnit(initialHardIronX, initialHardIronY, initialHardIronZ,
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
        if (running) {
            throw new LockedException();
        }

        initialHardIronX = convertMagneticFluxDensity(initialHardIron.getValueX(), initialHardIron.getUnit());
        initialHardIronY = convertMagneticFluxDensity(initialHardIron.getValueY(), initialHardIron.getUnit());
        initialHardIronZ = convertMagneticFluxDensity(initialHardIron.getValueZ(), initialHardIron.getUnit());
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
     * Gets initial hard-iron bias to be used to find a solution as an array.
     * Array values are expressed in Teslas (T).
     *
     * @return array containing coordinates of initial bias.
     */
    @Override
    public double[] getInitialHardIron() {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
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
        result[0] = initialHardIronX;
        result[1] = initialHardIronY;
        result[2] = initialHardIronZ;
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
        if (running) {
            throw new LockedException();
        }

        if (initialHardIron.length != BodyMagneticFluxDensity.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        initialHardIronX = initialHardIron[0];
        initialHardIronY = initialHardIron[1];
        initialHardIronZ = initialHardIron[2];
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
        result.setElementAtIndex(0, initialHardIronX);
        result.setElementAtIndex(1, initialHardIronY);
        result.setElementAtIndex(2, initialHardIronZ);
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
        if (running) {
            throw new LockedException();
        }
        if (initialHardIron.getRows() != BodyMagneticFluxDensity.COMPONENTS || initialHardIron.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        initialHardIronX = initialHardIron.getElementAtIndex(0);
        initialHardIronY = initialHardIron.getElementAtIndex(1);
        initialHardIronZ = initialHardIron.getElementAtIndex(2);
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
    public Collection<StandardDeviationBodyMagneticFluxDensity> getMeasurements() {
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
    public void setMeasurements(
            final Collection<StandardDeviationBodyMagneticFluxDensity> measurements) throws LockedException {
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
        return false;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement or not.
     *
     * @return true if quality scores are required, false otherwise.
     */
    @Override
    public boolean isQualityScoresRequired() {
        return false;
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
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final L listener) throws LockedException {
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
        return measurements != null && measurements.size() >= getMinimumRequiredMeasurements();
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
     * Estimates magnetometer calibration parameters containing scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if calibration fails for numerical reasons.
     */
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (running) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            running = true;

            onBeforeCalibrate();

            if (listener != null) {
                //noinspection unchecked
                listener.onCalibrateStart((C) this);
            }

            if (commonAxisUsed) {
                calibrateCommonAxis();
            } else {
                calibrateGeneral();
            }

            if (listener != null) {
                //noinspection unchecked
                listener.onCalibrateEnd((C) this);
            }

        } catch (final AlgebraException | FittingException | com.irurueta.numerical.NotReadyException e) {
            throw new CalibrationException(e);
        } finally {
            running = false;
        }
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
        return estimatedHardIron;
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
        if (estimatedHardIron != null) {
            System.arraycopy(estimatedHardIron, 0, result, 0, estimatedHardIron.length);
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
        return estimatedHardIron != null ? Matrix.newFromArray(estimatedHardIron) : null;
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
        if (estimatedHardIron != null) {
            result.fromArray(estimatedHardIron);
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
        return estimatedHardIron != null ? estimatedHardIron[0] : null;
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
        return estimatedHardIron != null ? estimatedHardIron[1] : null;
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
        return estimatedHardIron != null ? estimatedHardIron[2] : null;
    }

    /**
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @return x coordinate of estimated magnetometer bias.
     */
    @Override
    public MagneticFluxDensity getEstimatedHardIronXAsMagneticFluxDensity() {
        return estimatedHardIron != null
                ? new MagneticFluxDensity(estimatedHardIron[0], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets x coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronXAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedHardIron != null) {
            result.setValue(estimatedHardIron[0]);
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
        return estimatedHardIron != null
                ? new MagneticFluxDensity(estimatedHardIron[1], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets y coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronYAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedHardIron != null) {
            result.setValue(estimatedHardIron[1]);
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
        return estimatedHardIron != null
                ? new MagneticFluxDensity(estimatedHardIron[2], MagneticFluxDensityUnit.TESLA) : null;
    }

    /**
     * Gets z coordinate of estimated magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if estimated magnetometer bias is available, false otherwise.
     */
    @Override
    public boolean getEstimatedHardIronZAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedHardIron != null) {
            result.setValue(estimatedHardIron[2]);
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
        return estimatedHardIron != null
                ? new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                estimatedHardIron[0], estimatedHardIron[1], estimatedHardIron[2])
                : null;
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
        if (estimatedHardIron != null) {
            result.setValueCoordinatesAndUnit(estimatedHardIron[0], estimatedHardIron[1], estimatedHardIron[2],
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
     * Gets variance of estimated x coordinate of magnetometer bias expressed in
     * squared Teslas (T^2).
     *
     * @return variance of estimated x coordinate of magnetometer bias or null if
     * not available.
     */
    public Double getEstimatedHardIronXVariance() {
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronXStandardDeviation() {
        final var variance = getEstimatedHardIronXVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated x coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity() {
        return estimatedCovariance != null
                ? new MagneticFluxDensity(getEstimatedHardIronXStandardDeviation(), MagneticFluxDensityUnit.TESLA)
                : null;
    }

    /**
     * Gets standard deviation of estimated x coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated x coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (estimatedCovariance != null) {
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
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronYStandardDeviation() {
        final var variance = getEstimatedHardIronYVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated y coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity() {
        return estimatedCovariance != null
                ? new MagneticFluxDensity(getEstimatedHardIronYStandardDeviation(), MagneticFluxDensityUnit.TESLA)
                : null;
    }

    /**
     * Gets standard deviation of estimated y coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated y coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(
            final MagneticFluxDensity result) {
        if (estimatedCovariance != null) {
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
        return estimatedCovariance != null ? estimatedCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias
     * expressed in Teslas (T).
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public Double getEstimatedHardIronZStandardDeviation() {
        final var variance = getEstimatedHardIronZVariance();
        return variance != null ? Math.sqrt(variance) : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @return standard deviation of estimated z coordinate of magnetometer bias
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity() {
        return estimatedCovariance != null
                ? new MagneticFluxDensity(getEstimatedHardIronZStandardDeviation(), MagneticFluxDensityUnit.TESLA)
                : null;
    }

    /**
     * Gets standard deviation of estimated z coordinate of magnetometer bias.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of estimated z coordinate of
     * magnetometer bias is available, false otherwise.
     */
    public boolean getEstimatedHardIronZStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedCovariance != null) {
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
        return estimatedCovariance != null
                ? new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA,
                getEstimatedHardIronXStandardDeviation(),
                getEstimatedHardIronYStandardDeviation(),
                getEstimatedHardIronZStandardDeviation())
                : null;
    }

    /**
     * Gets standard deviation of estimated magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if standard deviation of magnetometer bias was available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviation(final MagneticFluxDensityTriad result) {
        if (estimatedCovariance != null) {
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
        return estimatedCovariance != null
                ? (getEstimatedHardIronXStandardDeviation() + getEstimatedHardIronYStandardDeviation()
                + getEstimatedHardIronZStandardDeviation()) / 3.0
                : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @return average of estimated standard deviation of magnetometer bias coordinates,
     * or null if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity() {
        return estimatedCovariance != null
                ? new MagneticFluxDensity(getEstimatedHardIronStandardDeviationAverage(), MagneticFluxDensityUnit.TESLA)
                : null;
    }

    /**
     * Gets average of estimated standard deviation of magnetometer bias coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if average of estimated standard deviation of magnetometer bias is available,
     * false otherwise.
     */
    public boolean getEstimatedHardIronStandardDeviationAverageAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedCovariance != null) {
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
        return estimatedCovariance != null
                ? Math.sqrt(getEstimatedHardIronXVariance() + getEstimatedHardIronYVariance()
                + getEstimatedHardIronZVariance()) : null;
    }

    /**
     * Gets norm of estimated standard deviation of magnetometer bias.
     *
     * @return norm of estimated standard deviation of magnetometer bias or null
     * if not available.
     */
    public MagneticFluxDensity getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity() {
        return estimatedCovariance != null
                ? new MagneticFluxDensity(getEstimatedHardIronStandardDeviationNorm(), MagneticFluxDensityUnit.TESLA)
                : null;
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
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationNorm());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Called before calibration occurs.
     * This can be overridden by subclasses.
     *
     * @throws CalibrationException if anything fails.
     */
    protected void onBeforeCalibrate() throws CalibrationException {
    }

    /**
     * Internally sets ground truth magnetic flux density norm to be expected at location where
     * measurements have been made, expressed in Teslas (T).
     *
     * @param groundTruthMagneticFluxDensityNorm ground truth magnetic flux density norm or null if undefined.
     * @throws IllegalArgumentException if provided value is negative.
     */
    protected void internalSetGroundTruthMagneticFluxDensityNorm(final Double groundTruthMagneticFluxDensityNorm) {
        if (groundTruthMagneticFluxDensityNorm != null && groundTruthMagneticFluxDensityNorm < 0.0) {
            throw new IllegalArgumentException();
        }
        this.groundTruthMagneticFluxDensityNorm = groundTruthMagneticFluxDensityNorm;
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    protected void setInputData() throws WrongSizeException {
        final var gtb = groundTruthMagneticFluxDensityNorm;
        final var gtb2 = gtb * gtb;

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, BodyMagneticFluxDensity.COMPONENTS);
        final var y = new double[numMeasurements];
        final var specificForceStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredMagneticFluxDensity = measurement.getMagneticFluxDensity();

            final var bmeasuredX = measuredMagneticFluxDensity.getBx();
            final var bmeasuredY = measuredMagneticFluxDensity.getBy();
            final var bmeasuredZ = measuredMagneticFluxDensity.getBz();

            x.setElementAt(i, 0, bmeasuredX);
            x.setElementAt(i, 1, bmeasuredY);
            x.setElementAt(i, 2, bmeasuredZ);

            y[i] = gtb2;

            specificForceStandardDeviations[i] = measurement.getMagneticFluxDensityStandardDeviation();

            i++;
        }

        fitter.setInputData(x, y, specificForceStandardDeviations);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws FittingException                         if Levenberg-Marquardt fails for numerical reasons.
     * @throws AlgebraException                         if there are numerical instabilities that prevent
     *                                                  matrix inversion.
     * @throws com.irurueta.numerical.NotReadyException never happens.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The magnetometer model is:
        // bmeas = ba + (I + Mm) * btrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // bmeas = ba + (I + Mm) * btrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // magnetometer model can be better expressed as:
        // bmeas = T*K*(btrue + b)
        // bmeas = M*(btrue + b)
        // bmeas = M*btrue + M*b

        // where:
        // M = I + Mm
        // bm = M*b = (I + Mm)*b --> b = M^-1*bm

        // We know that the norm of the true body magnetic flux density
        // is equal to the amount of Earth magnetic flux density at provided
        // position and timestamp
        // ||btrue|| = ||bEarth|| --> from 30 µT to 60 µT

        // Hence:
        // bmeas - M*b = M*btrue

        // M^-1 * (bmeas - M*b) = btrue

        // ||bEarth||^2 = ||btrue||^2 = (M^-1 * (bmeas - M*b))^T * (M^-1 * (bmeas - M*b))
        // ||bEarth||^2 = (bmeas - M*b)^T*(M^-1)^T * M^-1 * (bmeas - M*b)
        // ||bEarth||^2 = (bmeas - M * b)^T * ||M^-1||^2 * (bmeas - M * b)
        // ||bEarth||^2 = ||bmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        final var gradientEstimator = new GradientEstimator(this::evaluateGeneral);

        final var initialM = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        initialM.add(getInitialMm());

        final var invInitialM = Utils.inverse(initialM);
        final var initialBm = getInitialHardIronAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBm);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured magnetic flux density coordinates
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS];

                // biases b
                for (var i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                    initial[i] = initialB.getElementAtIndex(i);
                }

                // cross coupling errors M
                final var num = BodyMagneticFluxDensity.COMPONENTS * BodyMagneticFluxDensity.COMPONENTS;
                for (int i = 0, j = BodyMagneticFluxDensity.COMPONENTS; i < num; i++, j++) {
                    initial[j] = initialM.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives)
                    throws EvaluationException {

                bmeasX = point[0];
                bmeasY = point[1];
                bmeasZ = point[2];

                gradientEstimator.gradient(params, derivatives);

                return evaluateGeneral(params);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var bx = result[0];
        final var by = result[1];
        final var bz = result[2];

        final var m11 = result[3];
        final var m21 = result[4];
        final var m31 = result[5];

        final var m12 = result[6];
        final var m22 = result[7];
        final var m32 = result[8];

        final var m13 = result[9];
        final var m23 = result[10];
        final var m33 = result[11];

        final var mb = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        mm.setElementAtIndex(0, m11);
        mm.setElementAtIndex(1, m21);
        mm.setElementAtIndex(2, m31);

        mm.setElementAtIndex(3, m12);
        mm.setElementAtIndex(4, m22);
        mm.setElementAtIndex(5, m32);

        mm.setElementAtIndex(6, m13);
        mm.setElementAtIndex(7, m23);
        mm.setElementAtIndex(8, m33);

        setResult(mm, mb);

        // at this point covariance is expressed in terms of b and M, and must
        // be expressed in terms of ba and Ma.
        // We know that:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        // and that ba and Ma are expressed as:
        // Mm = M - I
        // bm = M * b

        // Mm = [m11 - 1    m12         m13    ] =  [sx     mxy     mxz]
        //      [m21        m22 - 1     m23    ]    [myx    sy      myz]
        //      [m31        m32         m33 - 1]    [mzx    mzy     sz ]

        // bm = [m11 * bx + m12 * by + m13 * bz] = 	[bmx]
        //      [m21 * bx + m22 * by + m23 * bz]	[bmy]
        //      [m31 * bx + m32 * by + m33 * bz]	[bmz]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33)
        // as:
        // [bmx] = [m11 * bx + m12 * by + m13 * bz]
        // [bmy]   [m21 * bx + m22 * by + m23 * bz]
        // [bmz]   [m31 * bx + m32 * by + m33 * bz]
        // [sx]	   [m11 - 1]
        // [sy]	   [m22 - 1]
        // [sz]	   [m33 -1]
        // [mxy]   [m12]
        // [mxz]   [m13]
        // [myx]   [m21]
        // [myz]   [m23]
        // [mzx]   [m31]
        // [mzy]   [m32]

        // Then the Jacobian of F(b, M) is:
        // J = 	[m11  m12  m13  bx  0   0   by  0   0   bz  0   0 ]
        //	    [m21  m22  m23  0   bx  0   0   by  0   0   bz  0 ]
        //	    [m31  m32  m33  0   0   bx  0   0   by  0   0   bz]
        //	    [0    0    0    1   0   0   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   1   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   0   0   1 ]
        //	    [0    0    0    0   0   0   1   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   1   0   0 ]
        //	    [0    0    0    0   1   0   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0   0   1   0 ]
        //	    [0    0    0    0   0   1   0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   1   0   0   0 ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS, GENERAL_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(1, 0, m21);
        jacobian.setElementAt(2, 0, m31);

        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(2, 1, m32);

        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(2, 2, m33);

        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(3, 3, 1.0);

        jacobian.setElementAt(1, 4, bx);
        jacobian.setElementAt(8, 4, 1.0);

        jacobian.setElementAt(2, 5, bx);
        jacobian.setElementAt(10, 5, 1.0);

        jacobian.setElementAt(0, 6, by);
        jacobian.setElementAt(6, 6, 1.0);

        jacobian.setElementAt(1, 7, by);
        jacobian.setElementAt(4, 7, 1.0);

        jacobian.setElementAt(2, 8, by);
        jacobian.setElementAt(11, 8, 1.0);

        jacobian.setElementAt(0, 9, bz);
        jacobian.setElementAt(7, 9, 1.0);

        jacobian.setElementAt(1, 10, bz);
        jacobian.setElementAt(9, 10, 1.0);

        jacobian.setElementAt(2, 11, bz);
        jacobian.setElementAt(5, 11, 1.0);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws FittingException                         if Levenberg-Marquardt fails for numerical reasons.
     * @throws AlgebraException                         if there are numerical instabilities that prevent
     *                                                  matrix inversion.
     * @throws com.irurueta.numerical.NotReadyException never happens.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The magnetometer model is:
        // bmeas = bm + (I + Mm) * btrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // bmeas = bm + (I + Mm) * btrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // magnetometer model can be better expressed as:
        // bmeas = T*K*(btrue + b)
        // bmeas = M*(btrue + b)
        // bmeas = M*btrue + M*b

        //where:
        // M = I + Mm
        // bm = M*b = (I + Mm)*b --> b = M^-1*bm

        // We know that the norm of the true body magnetic flux density
        // is equal to the amount of Earth magnetic flux density at provided
        // position and timestamp
        // ||btrue|| = ||bEarth|| --> from 30 µT to 60 µT

        // Hence:
        // bmeas - M*b = M*btrue

        // M^-1 * (bmeas - M*b) = btrue

        // ||bEarth||^2 = ||btrue||^2 = (M^-1 * (bmeas - M*b))^T * (M^-1 * (bmeas - M*b))
        // ||bEarth||^2 = (bmeas - M*b)^T*(M^-1)^T * M^-1 * (bmeas - M*b)
        // ||bEarth||^2 = (bmeas - M * b)^T * ||M^-1||^2 * (bmeas - M * b)
        // ||bEarth||^2 = ||bmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]


        final var gradientEstimator = new GradientEstimator(this::evaluateCommonAxis);

        final var initialM = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        initialM.add(getInitialMm());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final var invInitialM = Utils.inverse(initialM);
        final var initialBm = getInitialHardIronAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBm);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured magnetic flux density coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                // biases b
                for (var i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                    initial[i] = initialB.getElementAtIndex(i);
                }

                // upper diagonal cross coupling errors M
                var k = BodyMagneticFluxDensity.COMPONENTS;
                for (var j = 0; j < BodyMagneticFluxDensity.COMPONENTS; j++) {
                    for (var i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
                        if (i <= j) {
                            initial[k] = initialM.getElementAt(i, j);
                            k++;
                        }
                    }
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives)
                    throws EvaluationException {

                bmeasX = point[0];
                bmeasY = point[1];
                bmeasZ = point[2];

                gradientEstimator.gradient(params, derivatives);

                return evaluateCommonAxis(params);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var bx = result[0];
        final var by = result[1];
        final var bz = result[2];

        final var m11 = result[3];

        final var m12 = result[4];
        final var m22 = result[5];

        final var m13 = result[6];
        final var m23 = result[7];
        final var m33 = result[8];

        final var mb = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        mm.setElementAtIndex(0, m11);
        mm.setElementAtIndex(1, 0.0);
        mm.setElementAtIndex(2, 0.0);

        mm.setElementAtIndex(3, m12);
        mm.setElementAtIndex(4, m22);
        mm.setElementAtIndex(5, 0.0);

        mm.setElementAtIndex(6, m13);
        mm.setElementAtIndex(7, m23);
        mm.setElementAtIndex(8, m33);

        setResult(mm, mb);

        // at this point covariance is expressed in terms of b and M, and must
        // be expressed in terms of ba and Ma.
        // We know that:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0    	m22 	m23]
        //     [0    	0    	m33]

        // m21 = m31 = m32 = 0

        // and that ba and Ma are expressed as:
        // Ma = M - I
        // ba = M * b

        // Ma = [m11 - 1    m12         m13    ] =  [sx     mxy     mxz]
        //      [0          m22 - 1     m23    ]    [0      sy      myz]
        //      [0          0           m33 - 1]    [0      0       sz ]

        // ba = [m11 * bx + m12 * by + m13 * bz] = 	[bax]
        //      [           m22 * by + m23 * bz]	[bay]
        //      [                      m33 * bz]	[baz]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m12, m22, m13, m23, m33)
        // as:
        // [bax] = [m11 * bx + m12 * by + m13 * bz]
        // [bay]   [m22 * by + m23 * bz]
        // [baz]   [m33 * bz]
        // [sx]	   [m11 - 1]
        // [sy]	   [m22 - 1]
        // [sz]	   [m33 -1]
        // [mxy]   [m12]
        // [mxz]   [m13]
        // [myx]   [0]
        // [myz]   [m23]
        // [mzx]   [0]
        // [mzy]   [0]

        // Then the Jacobian of F(b, M) is:
        // J = 	[m11  m12  m13  bx  by  0   bz  0   0 ]
        //	    [0    m22  m23  0   0   by  0   bz  0 ]
        //	    [0    0    m33  0   0   0   0   0   bz]
        //	    [0    0    0    1   0   0   0   0   0 ]
        //	    [0    0    0    0   0   1   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   1 ]
        //	    [0    0    0    0   1   0   0   0   0 ]
        //	    [0    0    0    0   0   0   1   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   1   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]
        //	    [0    0    0    0   0   0   0   0   0 ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);

        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(1, 1, m22);

        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(2, 2, m33);

        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(3, 3, 1.0);

        jacobian.setElementAt(0, 4, by);
        jacobian.setElementAt(6, 4, 1.0);

        jacobian.setElementAt(1, 5, by);
        jacobian.setElementAt(4, 5, 1.0);

        jacobian.setElementAt(0, 6, bz);
        jacobian.setElementAt(7, 6, 1.0);

        jacobian.setElementAt(1, 7, bz);
        jacobian.setElementAt(9, 7, 1.0);

        jacobian.setElementAt(2, 8, bz);
        jacobian.setElementAt(5, 8, 1.0);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal cross-coupling matrix.
     * @param b internal bias matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m, final Matrix b) throws AlgebraException {
        // Because:
        // M = I + Mm
        // b = M^-1*bm

        // Then:
        // Mm = M - I
        // bm = M*b

        if (estimatedHardIron == null) {
            estimatedHardIron = new double[BodyMagneticFluxDensity.COMPONENTS];
        }

        final var bm = m.multiplyAndReturnNew(b);
        bm.toArray(estimatedHardIron);

        if (estimatedMm == null) {
            estimatedMm = m;
        } else {
            estimatedMm.copyFrom(m);
        }

        for (var i = 0; i < BodyMagneticFluxDensity.COMPONENTS; i++) {
            estimatedMm.setElementAt(i, i, estimatedMm.getElementAt(i, i) - 1.0);
        }

        estimatedCovariance = fitter.getCovar();
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters for the general case.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the general purpose case.
     *               Must have length 12.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneral(final double[] params) throws EvaluationException {
        final var bx = params[0];
        final var by = params[1];
        final var bz = params[2];

        final var m11 = params[3];
        final var m21 = params[4];
        final var m31 = params[5];

        final var m12 = params[6];
        final var m22 = params[7];
        final var m32 = params[8];

        final var m13 = params[9];
        final var m23 = params[10];
        final var m33 = params[11];

        return evaluate(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33);
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters when common z-axis is assumed.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the common z-axis case.
     *               Must have length 9.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxis(final double[] params) throws EvaluationException {
        final var bx = params[0];
        final var by = params[1];
        final var bz = params[2];

        final var m11 = params[3];

        final var m12 = params[4];
        final var m22 = params[5];

        final var m13 = params[6];
        final var m23 = params[7];
        final var m33 = params[8];

        return evaluate(bx, by, bz, m11, 0.0, 0.0, m12, m22, 0.0, m13, m23, m33);
    }

    /**
     * Computes estimated true magnetic flux density squared norm using current measured
     * body magnetic flux density and provided parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param bx  x-coordinate of bias.
     * @param by  y-coordinate of bias.
     * @param bz  z-coordinate of bias.
     * @param m11 element 1,1 of cross-coupling error matrix.
     * @param m21 element 2,1 of cross-coupling error matrix.
     * @param m31 element 3,1 of cross-coupling error matrix.
     * @param m12 element 1,2 of cross-coupling error matrix.
     * @param m22 element 2,2 of cross-coupling error matrix.
     * @param m32 element 3,2 of cross-coupling error matrix.
     * @param m13 element 1,3 of cross-coupling error matrix.
     * @param m23 element 2,3 of cross-coupling error matrix.
     * @param m33 element 3,3 of cross-coupling error matrix.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluate(final double bx, final double by, final double bz,
                            final double m11, final double m21, final double m31,
                            final double m12, final double m22, final double m32,
                            final double m13, final double m23, final double m33) throws EvaluationException {

        // bmeas = M*(btrue + b)

        // btrue = M^-1*bmeas - b

        try {
            if (bmeas == null) {
                bmeas = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
            }
            if (m == null) {
                m = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            }
            if (invM == null) {
                invM = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            }
            if (b == null) {
                b = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
            }
            if (btrue == null) {
                btrue = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
            }

            bmeas.setElementAtIndex(0, bmeasX);
            bmeas.setElementAtIndex(1, bmeasY);
            bmeas.setElementAtIndex(2, bmeasZ);

            m.setElementAt(0, 0, m11);
            m.setElementAt(1, 0, m21);
            m.setElementAt(2, 0, m31);

            m.setElementAt(0, 1, m12);
            m.setElementAt(1, 1, m22);
            m.setElementAt(2, 1, m32);

            m.setElementAt(0, 2, m13);
            m.setElementAt(1, 2, m23);
            m.setElementAt(2, 2, m33);

            Utils.inverse(m, invM);

            b.setElementAtIndex(0, bx);
            b.setElementAtIndex(1, by);
            b.setElementAtIndex(2, bz);

            invM.multiply(bmeas, btrue);
            btrue.subtract(b);

            final var norm = Utils.normF(btrue);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
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
