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
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Abstract class to estimate accelerometer biases, cross couplings and scaling factors
 * when gravity norm is known (either because it has been directly provided or because
 * position respect Earth is known, and thus gravity norm is also known).
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 10 measurements taken at a single unknown position must
 * be taken at 10 different unknown orientations and zero velocity when common z-axis is
 * assumed, otherwise at least 13 measurements are required.
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
 *
 * @param <C> a calibrator type.
 * @param <L> a listener type.
 */
public abstract class BaseGravityNormAccelerometerCalibrator<C extends BaseGravityNormAccelerometerCalibrator<?, ?>,
        L extends BaseGravityNormAccelerometerCalibratorListener<C>> implements AccelerometerNonLinearCalibrator,
        UnknownBiasNonLinearAccelerometerCalibrator, AccelerometerCalibrationSource,
        AccelerometerBiasUncertaintySource, UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    public static final int GENERAL_UNKNOWNS = 12;

    /**
     * Required minimum number of measurements when common z-axis is assumed.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS = COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL = GENERAL_UNKNOWNS + 1;

    /**
     * Ground truth gravity norm to be expected at location where measurements have been made,
     * expressed in meters per squared second (m/s^2).
     */
    protected Double groundTruthGravityNorm;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter fitter = new LevenbergMarquardtMultiDimensionFitter();

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
     * Contains a collection of body kinematics measurements taken at
     * a the same position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     */
    private Collection<StandardDeviationBodyKinematics> measurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private L listener;

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
     * Indicates whether estimator is running.
     */
    private boolean running;

    /**
     * Internally holds x-coordinate of measured specific force during calibration.
     */
    private double fmeasX;

    /**
     * Internally holds y-coordinate of measured specific force during calibration.
     */
    private double fmeasY;

    /**
     * Internally holds z-coordinate of measured specific force during calibration.
     */
    private double fmeasZ;

    /**
     * Internally holds measured specific force during calibration expressed as
     * a column matrix.
     */
    private Matrix fmeas;

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
     * Internally holds computed true specific force during calibration.
     */
    private Matrix ftrue;

    /**
     * Constructor.
     */
    protected BaseGravityNormAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(final L listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Collection<StandardDeviationBodyKinematics> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final L listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected BaseGravityNormAccelerometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final L listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final L listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) {
        try {
            setInitialBias(initialBiasX, initialBiasY, initialBiasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        try {
            setInitialBias(initialBiasX, initialBiasY, initialBiasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz, initialMxy, initialMxz,
                    initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution. This is expressed in meters per squared
     *                     second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution. This is expressed in meters per squared
     *                       second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz, initialMxy, initialMxz,
                    initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param initialMxy   initial x-y cross coupling error.
     * @param initialMxz   initial x-z cross coupling error.
     * @param initialMyx   initial y-x cross coupling error.
     * @param initialMyz   initial y-z cross coupling error.
     * @param initialMzx   initial z-x cross coupling error.
     * @param initialMzy   initial z-y cross coupling error.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param initialMxy     initial x-y cross coupling error.
     * @param initialMxz     initial x-z cross coupling error.
     * @param initialMyx     initial y-x cross coupling error.
     * @param initialMyz     initial y-z cross coupling error.
     * @param initialMzx     initial z-x cross coupling error.
     * @param initialMzy     initial z-y cross coupling error.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(final double[] initialBias) {
        try {
            setInitialBias(initialBias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialBias initial accelerometer bias to be used to find a solution.
     *                    This must have length 3 and is expressed in meters per
     *                    squared second (m/s^2).
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(final double[] initialBias, final L listener) {
        this(initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias) {
        this(initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final L listener) {
        this(measurements, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final double[] initialBias) {
        this(initialBias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] initialBias, final L listener) {
        this(commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias) {
        this(commonAxisUsed, initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final L listener) {
        this(measurements, commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Matrix initialBias) {
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
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Matrix initialBias, final L listener) {
        this(initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias) {
        this(initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final L listener) {
        this(measurements, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final Matrix initialBias) {
        this(initialBias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias, final L listener) {
        this(commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias) {
        this(commonAxisUsed, initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at the same position with zero velocity
     *                       and unknown different orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final L listener) {
        this(measurements, commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Matrix initialBias, final Matrix initialMa) {
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
     * @param initialBias initial bias to find a solution.
     * @param initialMa   initial scale factors and cross coupling errors matrix.
     * @param listener    listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMa, final L listener) {
        this(measurements, initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(commonAxisUsed, initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa) {
        this(commonAxisUsed, initialBias, initialMa);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final L listener) {
        this(groundTruthGravityNorm);
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
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements) {
        this(groundTruthGravityNorm);
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
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final L listener) {
        this(measurements, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed) {
        this(commonAxisUsed);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final L listener) {
        this(commonAxisUsed, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        this(measurements, commonAxisUsed);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final L listener) {
        this(measurements, commonAxisUsed, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double initialBiasX,
            final double initialBiasY, final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution. This is expressed in meters per squared
     *                               second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
                initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param measurements           collection of body kinematics measurements with standard
     *                               deviations taken at the same position with zero velocity
     *                               and unknown different orientations.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * @param initialBiasX           initial x-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasY           initial y-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialBiasZ           initial z-coordinate of accelerometer bias to be used
     *                               to find a solution.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param initialMxy             initial x-y cross coupling error.
     * @param initialMxz             initial x-z cross coupling error.
     * @param initialMyx             initial y-x cross coupling error.
     * @param initialMyz             initial y-z cross coupling error.
     * @param initialMzx             initial z-x cross coupling error.
     * @param initialMzy             initial z-y cross coupling error.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final double[] initialBias) {
        this(initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final double[] initialBias,
                                                     final L listener) {
        this(initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias) {
        this(measurements, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] initialBias, final L listener) {
        this(measurements, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double[] initialBias) {
        this(commonAxisUsed, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial accelerometer bias to be used to find a solution.
     *                               This must have length 3 and is expressed in meters per
     *                               squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double[] initialBias, final L listener) {
        this(commonAxisUsed, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(measurements, commonAxisUsed, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias, final L listener) {
        this(measurements, commonAxisUsed, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final Matrix initialBias) {
        this(initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final Matrix initialBias,
                                                     final L listener) {
        this(initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias) {
        this(measurements, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final L listener) {
        this(measurements, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix initialBias) {
        this(commonAxisUsed, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix initialBias,
            final L listener) {
        this(commonAxisUsed, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(measurements, commonAxisUsed, initialBias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final L listener) {
        this(measurements, commonAxisUsed, initialBias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(initialBias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(measurements, initialBias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(measurements, initialBias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa) {
        this(commonAxisUsed, initialBias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param initialBias            initial bias to find a solution.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix initialBias,
            final Matrix initialMa, final L listener) {
        this(commonAxisUsed, initialBias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        this(measurements, commonAxisUsed, initialBias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
    protected BaseGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, initialBias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
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
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasX() {
        return initialBiasX;
    }

    /**
     * Sets initial x-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
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
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasY() {
        return initialBiasY;
    }

    /**
     * Sets initial y-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
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
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    @Override
    public double getInitialBiasZ() {
        return initialBiasZ;
    }

    /**
     * Sets initial z-coordinate of accelerometer bias to be used to find a solution.
     * This is expressed in meters per squared second (m/s^2).
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
     *
     * @return initial x-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasXAsAcceleration() {
        return new Acceleration(initialBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial x-coordinate of accelerometer bias to be used to find a solution.
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
     *
     * @return initial y-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasYAsAcceleration() {
        return new Acceleration(initialBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial y-coordinate of accelerometer bias to be used to find a solution.
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
     *
     * @return initial z-coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getInitialBiasZAsAcceleration() {
        return new Acceleration(initialBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets initial z-coordinate of accelerometer bias to be used to find a solution.
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
     *
     * @param initialBiasX initial x-coordinate of accelerometer bias.
     * @param initialBiasY initial y-coordinate of accelerometer bias.
     * @param initialBiasZ initial z-coordinate of accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(final Acceleration initialBiasX, final Acceleration initialBiasY,
                               final Acceleration initialBiasZ) throws LockedException {
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
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setInitialBias(final AccelerationTriad initialBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        initialBiasX = convertAcceleration(initialBias.getValueX(), initialBias.getUnit());
        initialBiasY = convertAcceleration(initialBias.getValueY(), initialBias.getUnit());
        initialBiasZ = convertAcceleration(initialBias.getValueZ(), initialBias.getUnit());
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
     * Gets initial bias to be used to find a solution as an array.
     * Array values are expressed in meters per squared second (m/s^2).
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
     * Sets initial bias to be used to find a solution as a column matrix with
     * values expressed in meters per squared second (m/s^2).
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
     * Gets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @return collection of body kinematics measurements at a known position
     * with unknown orientations.
     */
    @Override
    public Collection<StandardDeviationBodyKinematics> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     *
     * @param measurements collection of body kinematics measurements at a
     *                     known position with unknown orientations.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(final Collection<StandardDeviationBodyKinematics> measurements)
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
    public L getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
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
     * biases.
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
        return estimatedBiases != null ?
                new Acceleration(estimatedBiases[0], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null ?
                new Acceleration(estimatedBiases[1], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null ?
                new Acceleration(estimatedBiases[2], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null ?
                new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                        estimatedBiases[0], estimatedBiases[1], estimatedBiases[2]) : null;
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
                ? new Acceleration(getEstimatedBiasFxStandardDeviation(), AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
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
        return estimatedCovariance != null ?
                new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                        getEstimatedBiasFxStandardDeviation(),
                        getEstimatedBiasFyStandardDeviation(),
                        getEstimatedBiasFzStandardDeviation()) : null;
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
        return estimatedCovariance != null ?
                (getEstimatedBiasFxStandardDeviation() + getEstimatedBiasFyStandardDeviation()
                        + getEstimatedBiasFzStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null.
     */
    public Acceleration getEstimatedBiasStandardDeviationAverageAsAcceleration() {
        return estimatedCovariance != null ?
                new Acceleration(getEstimatedBiasStandardDeviationAverage(),
                        AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    protected static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(), acceleration.getUnit());
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
    protected void internalSetGroundTruthGravityNorm(final Double groundTruthGravityNorm) {
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
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    private void setInputData() throws WrongSizeException {

        final var g = groundTruthGravityNorm;
        final var g2 = g * g;

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final var y = new double[numMeasurements];
        final var specificForceStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredKinematics = measurement.getKinematics();

            final var fx = measuredKinematics.getFx();
            final var fy = measuredKinematics.getFy();
            final var fz = measuredKinematics.getFz();

            x.setElementAt(i, 0, fx);
            x.setElementAt(i, 1, fy);
            x.setElementAt(i, 2, fz);

            y[i] = g2;

            specificForceStandardDeviations[i] = measurement.getSpecificForceStandardDeviation();

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
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // accelerometer model can be better expressed as:
        // fmeas = T*K*(ftrue + b)
        // fmeas = M*(ftrue + b)
        // fmeas = M*ftrue + M*b

        // where:
        // M = I + Ma
        // ba = M*b = (I + Ma)*b --> b = M^-1*ba

        // We know that the norm of the true specific force is equal to the amount
        // of gravity at a certain Earth position
        // ||ftrue|| = ||g|| ~ 9.81 m/s^2

        // Hence:
        // fmeas - M*b = M*ftrue

        // M^-1 * (fmeas - M*b) = ftrue

        // ||g||^2 = ||ftrue||^2 = (M^-1 * (fmeas - M*b))^T * (M^-1 * (fmeas - M*b))
        // ||g||^2 = (fmeas - M*b)^T*(M^-1)^T * M^-1 * (fmeas - M*b)
        // ||g||^2 = (fmeas - M * b)^T * ||M^-1||^2 * (fmeas - M * b)
        // ||g||^2 = ||fmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        final var gradientEstimator = new GradientEstimator(this::evaluateGeneral);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        final var invInitialM = Utils.inverse(initialM);
        final var initialBa = getInitialBiasAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBa);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS];

                // biases b
                for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
                    initial[i] = initialB.getElementAtIndex(i);
                }

                // cross coupling errors M
                final var num = BodyKinematics.COMPONENTS * BodyKinematics.COMPONENTS;
                for (int i = 0, j = BodyKinematics.COMPONENTS; i < num; i++, j++) {
                    initial[j] = initialM.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives)
                    throws EvaluationException {

                fmeasX = point[0];
                fmeasY = point[1];
                fmeasZ = point[2];

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

        final var bias = new Matrix(BodyKinematics.COMPONENTS, 1);
        bias.setElementAtIndex(0, bx);
        bias.setElementAtIndex(1, by);
        bias.setElementAtIndex(2, bz);

        final var crossCoupling = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        crossCoupling.setElementAtIndex(0, m11);
        crossCoupling.setElementAtIndex(1, m21);
        crossCoupling.setElementAtIndex(2, m31);

        crossCoupling.setElementAtIndex(3, m12);
        crossCoupling.setElementAtIndex(4, m22);
        crossCoupling.setElementAtIndex(5, m32);

        crossCoupling.setElementAtIndex(6, m13);
        crossCoupling.setElementAtIndex(7, m23);
        crossCoupling.setElementAtIndex(8, m33);

        setResult(crossCoupling, bias);

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
        // Ma = M - I
        // ba = M * b

        // Ma = [m11 - 1    m12         m13    ] =  [sx     mxy     mxz]
        //      [m21        m22 - 1     m23    ]    [myx    sy      myz]
        //      [m31        m32         m33 - 1]    [mzx    mzy     sz ]

        // ba = [m11 * bx + m12 * by + m13 * bz] = 	[bax]
        //      [m21 * bx + m22 * by + m23 * bz]	[bay]
        //      [m31 * bx + m32 * by + m33 * bz]	[baz]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33)
        // as:
        // [bax] = 	[m11 * bx + m12 * by + m13 * bz]
        // [bay]	[m21 * bx + m22 * by + m23 * bz]
        // [baz]	[m31 * bx + m32 * by + m33 * bz]
        // [sx]		[m11 - 1]
        // [sy]		[m22 - 1]
        // [sz]		[m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [m21]
        // [myz]    [m23]
        // [mzx]    [m31]
        // [mzy]    [m32]

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
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // accelerometer model can be better expressed as:
        // fmeas = T*K*(ftrue + b)
        // fmeas = M*(ftrue + b)
        // fmeas = M*ftrue + M*b

        // where:
        // M = I + Ma
        // ba = M*b = (I + Ma)*b --> b = M^-1*ba

        // We know that the norm of the true specific force is equal to the amount
        // of gravity at a certain Earth position
        // ||ftrue|| = ||g|| ~ 9.81 m/s^2

        // Hence:
        // fmeas - M*b = M*ftrue

        // M^-1 * (fmeas - M*b) = ftrue

        // ||g||^2 = ||ftrue||^2 = (M^-1 * (fmeas - M*b))^T * (M^-1 * (fmeas - M*b))
        // ||g||^2 = (fmeas - M*b)^T*(M^-1)^T * M^-1 * (fmeas - M*b)
        // ||g||^2 = (fmeas - M * b)^T * ||M^-1||^2 * (fmeas - M * b)
        // ||g||^2 = ||fmeas - M * b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]


        final var gradientEstimator = new GradientEstimator(this::evaluateCommonAxis);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final var invInitialM = Utils.inverse(initialM);
        final var initialBa = getInitialBiasAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBa);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                // biases b
                for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
                    initial[i] = initialB.getElementAtIndex(i);
                }

                // upper diagonal cross coupling errors M
                var k = BodyKinematics.COMPONENTS;
                for (var j = 0; j < BodyKinematics.COMPONENTS; j++) {
                    for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
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

                fmeasX = point[0];
                fmeasY = point[1];
                fmeasZ = point[2];

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

        final var bias = new Matrix(BodyKinematics.COMPONENTS, 1);
        bias.setElementAtIndex(0, bx);
        bias.setElementAtIndex(1, by);
        bias.setElementAtIndex(2, bz);

        final var crossCoupling = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        crossCoupling.setElementAtIndex(0, m11);
        crossCoupling.setElementAtIndex(1, 0.0);
        crossCoupling.setElementAtIndex(2, 0.0);

        crossCoupling.setElementAtIndex(3, m12);
        crossCoupling.setElementAtIndex(4, m22);
        crossCoupling.setElementAtIndex(5, 0.0);

        crossCoupling.setElementAtIndex(6, m13);
        crossCoupling.setElementAtIndex(7, m23);
        crossCoupling.setElementAtIndex(8, m33);

        setResult(crossCoupling, bias);

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
        // [bax] = 	[m11 * bx + m12 * by + m13 * bz]
        // [bay]	[m22 * by + m23 * bz]
        // [baz]	[m33 * bz]
        // [sx]		[m11 - 1]
        // [sy]		[m22 - 1]
        // [sz]		[m33 -1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [0]
        // [myz]    [m23]
        // [mzx]    [0]
        // [mzy]    [0]

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
        // M = I + Ma
        // b = M^-1*ba

        // Then:
        // Ma = M - I
        // ba = M*b

        if (estimatedBiases == null) {
            estimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        final var ba = m.multiplyAndReturnNew(b);
        ba.toArray(estimatedBiases);

        if (estimatedMa == null) {
            estimatedMa = m;
        } else {
            estimatedMa.copyFrom(m);
        }

        for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
            estimatedMa.setElementAt(i, i, estimatedMa.getElementAt(i, i) - 1.0);
        }

        estimatedCovariance = fitter.getCovar();
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters for the general case.
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
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters when common z-axis is assumed.
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
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters.
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

        // fmeas = M*(ftrue + b)

        // ftrue = M^-1*fmeas - b

        try {
            if (fmeas == null) {
                fmeas = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (m == null) {
                m = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }
            if (invM == null) {
                invM = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }
            if (b == null) {
                b = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (ftrue == null) {
                ftrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            fmeas.setElementAtIndex(0, fmeasX);
            fmeas.setElementAtIndex(1, fmeasY);
            fmeas.setElementAtIndex(2, fmeasZ);

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

            invM.multiply(fmeas, ftrue);
            ftrue.subtract(b);

            final var norm = Utils.normF(ftrue);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }
}
