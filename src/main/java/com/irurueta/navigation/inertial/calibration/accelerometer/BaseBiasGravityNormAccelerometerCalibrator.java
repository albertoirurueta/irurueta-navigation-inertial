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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
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
 * Abstract class to estimate accelerometer cross couplings and scaling factors when
 * accelerometer biases and gravity norm are known.
 * Gravity norm can be known because it has been directly provided or because position
 * respect Earth is known, and thus gravity norm can be computed.
 * This calibrator uses Levenberg-Marquardt to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single known position must
 * be taken at 7 different unknown orientations and zero velocity when common z-axis
 * is assumed, otherwise at least 10 measurements are required.
 * <p>
 * Measured specific force is assumed to follow the model shown below:
 * <pre>
 *     fmeas = ba + (I + Ma) * ftrue + w
 * </pre>
 * Where:
 * - fmeas is the measured specific force. This is a 3x1 vector.
 * - ba is accelerometer bias, which is known. This is a 3x1 vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-truth specific force.
 * - w is measurement noise.
 *
 * @param <C> a calibrator type.
 * @param <L> a listener type.
 */
public abstract class BaseBiasGravityNormAccelerometerCalibrator<
        C extends BaseBiasGravityNormAccelerometerCalibrator<?, ?>,
        L extends BaseBiasGravityNormAccelerometerCalibratorListener<C>> implements AccelerometerNonLinearCalibrator,
        KnownBiasAccelerometerCalibrator, UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case.
     */
    public static final int GENERAL_UNKNOWNS = 9;

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
     * X-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double biasX;

    /**
     * Y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     */
    private double biasY;

    /**
     * Z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
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
     * Contains a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
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
     * Estimated covariance matrix for estimated position.
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
     * Internally holds biases during calibration in internal format.
     */
    private Matrix b;

    /**
     * Internally holds computed true specific force during calibration.
     */
    private Matrix ftrue;

    /**
     * Internally hold biases during calibration in external format.
     */
    private Matrix ba;

    /**
     * Constructor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final L listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements) {
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final L listener) {
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final L listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasY y-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     * @param biasZ z-coordinate of accelerometer bias.
     *              This is expressed in meters per squared second (m/s^2).
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final double biasX, final double biasY, final double biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasY    y-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param biasZ    z-coordinate of accelerometer bias.
     *                 This is expressed in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final double biasX, final double biasY, final double biasZ,
                                                         final L listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed,
                                                         final double biasX, final double biasY, final double biasZ,
                                                         final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX x-coordinate of accelerometer bias.
     * @param biasY y-coordinate of accelerometer bias.
     * @param biasZ z-coordinate of accelerometer bias.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    x-coordinate of accelerometer bias.
     * @param biasY    y-coordinate of accelerometer bias.
     * @param biasZ    z-coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasY     y-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param biasZ     z-coordinate of accelerometer bias.
     *                  This is expressed in meters per squared second (m/s^2).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ);
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX     x-coordinate of accelerometer bias.
     * @param biasY     y-coordinate of accelerometer bias.
     * @param biasZ     z-coordinate of accelerometer bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasY      y-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param biasZ      z-coordinate of accelerometer bias.
     *                   This is expressed in meters per squared second (m/s^2).
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ);
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
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasY        y-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
     * @param biasZ        z-coordinate of accelerometer bias.
     *                     This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param biasX          x-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasY          y-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
     * @param biasZ          z-coordinate of accelerometer bias.
     *                       This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ);
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
     * @param biasX      x-coordinate of accelerometer bias.
     * @param biasY      y-coordinate of accelerometer bias.
     * @param biasZ      z-coordinate of accelerometer bias.
     * @param initialSx  initial x scaling factor.
     * @param initialSy  initial y scaling factor.
     * @param initialSz  initial z scaling factor.
     * @param initialMxy initial x-y cross coupling error.
     * @param initialMxz initial x-z cross coupling error.
     * @param initialMyx initial y-x cross coupling error.
     * @param initialMyz initial y-z cross coupling error.
     * @param initialMzx initial z-x cross coupling error.
     * @param initialMzy initial z-y cross coupling error.
     * @param listener   listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param biasX        x-coordinate of accelerometer bias.
     * @param biasY        y-coordinate of accelerometer bias.
     * @param biasZ        z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param biasX          x-coordinate of accelerometer bias.
     * @param biasY          y-coordinate of accelerometer bias.
     * @param biasZ          z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias. This must have length 3 and is expressed
     *             in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final double[] bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias. This must have length 3 and is expressed
     *                 in meters per squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final double[] bias, final L listener) {
        this(bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] bias) {
        this(bias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias. This must have length 3 and is expressed
     *                     in meters per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] bias, final L listener) {
        this(measurements, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final double[] bias) {
        this(bias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final double[] bias, final L listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias. This must have length 3 and is expressed
     *                       in meters per squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias, final L listener) {
        this(measurements, commonAxisUsed, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Matrix bias) {
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Matrix bias, final L listener) {
        this(bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix bias) {
        this(bias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix bias, final L listener) {
        this(measurements, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final Matrix bias) {
        this(bias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final L listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known accelerometer bias.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final L listener) {
        this(measurements, commonAxisUsed, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Matrix bias, final Matrix initialMa) {
        this(bias);
        try {
            setInitialMa(initialMa);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known accelerometer bias.
     * @param initialMa initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Matrix bias, final Matrix initialMa, final L listener) {
        this(bias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at the same position with zero velocity
     *                     and unknown different orientations.
     * @param bias         known accelerometer bias.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa, final L listener) {
        this(measurements, bias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final boolean commonAxisUsed, final Matrix bias,
                                                         final Matrix initialMa) {
        this(bias, initialMa);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMa, final L listener) {
        this(commonAxisUsed, bias, initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix bias, final Matrix initialMa) {
        this(commonAxisUsed, bias, initialMa);
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
     * @param bias           known accelerometer bias.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix bias, final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, bias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm) {
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
    protected BaseBiasGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final L listener) {
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
     * @throws IllegalArgumentException if provided gravity norm value is negative.     *
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double biasX, final double biasY, final double biasZ,
            final L listener) {
        this(biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(measurements, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(measurements, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(measurements, biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy, final double initialSz,
            final L listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ, final double initialSx, final double initialSy,
            final double initialSz, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
     * @param initialSx              initial x scaling factor.
     * @param initialSy              initial y scaling factor.
     * @param initialSz              initial z scaling factor.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double biasX, final double biasY,
            final double biasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasY                  y-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
     * @param biasZ                  z-coordinate of accelerometer bias.
     *                               This is expressed in meters per squared second (m/s^2).
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz, final double initialMzx,
            final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Acceleration biasX, final Acceleration biasY,
            final Acceleration biasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx, initialMyz,
                initialMzx, initialMzy, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final double initialSx,
            final double initialSy, final double initialSz, final double initialMxy,
            final double initialMxz, final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ, final double initialSx, final double initialSy,
            final double initialSz, final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Acceleration biasX,
            final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy, listener);
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
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy,
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
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param biasX                  x-coordinate of accelerometer bias.
     * @param biasY                  y-coordinate of accelerometer bias.
     * @param biasZ                  z-coordinate of accelerometer bias.
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
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy, final L listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy,
                initialMxz, initialMyx, initialMyz, initialMzx, initialMzy, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final double[] bias) {
        this(bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final double[] bias, final L listener) {
        this(bias, listener);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias) {
        this(measurements, bias);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final double[] bias, final L listener) {
        this(measurements, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double[] bias) {
        this(commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final double[] bias, final L listener) {
        this(commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias) {
        this(measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias. This must have length 3 and is expressed
     *                               in meters per squared second (m/s^2).
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] bias, final L listener) {
        this(measurements, commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(final Double groundTruthGravityNorm, final Matrix bias) {
        this(bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Matrix bias, final L listener) {
        this(bias, listener);
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
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias) {
        this(measurements, bias);
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
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final L listener) {
        this(measurements, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix bias) {
        this(commonAxisUsed, bias);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix bias, final L listener) {
        this(commonAxisUsed, bias, listener);
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
     * @param bias                   known accelerometer bias.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias) {
        this(measurements, commonAxisUsed, bias);
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
     * @param bias                   known accelerometer bias.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final L listener) {
        this(measurements, commonAxisUsed, bias, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Matrix bias, final Matrix initialMa) {
        this(bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Matrix bias, final Matrix initialMa, final L listener) {
        this(bias, initialMa, listener);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa) {
        this(measurements, bias, initialMa);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMa, final L listener) {
        this(measurements, bias, initialMa, listener);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa) {
        this(commonAxisUsed, bias, initialMa);
        internalSetGroundTruthGravityNorm(groundTruthGravityNorm);
    }

    /**
     * Constructor.
     *
     * @param groundTruthGravityNorm ground truth gravity norm expressed in meters per
     *                               squared second (m/s^2).
     * @param commonAxisUsed         indicates whether z-axis is assumed to be common for
     *                               accelerometer and gyroscope.
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final boolean commonAxisUsed, final Matrix bias,
            final Matrix initialMa, final L listener) {
        this(commonAxisUsed, bias, initialMa, listener);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMa) {
        this(measurements, commonAxisUsed, bias, initialMa);
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
     * @param bias                   known accelerometer bias.
     * @param initialMa              initial scale factors and cross coupling errors matrix.
     * @param listener               listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3 or
     *                                  if provided gravity norm value is negative.
     */
    protected BaseBiasGravityNormAccelerometerCalibrator(
            final Double groundTruthGravityNorm, final Collection<StandardDeviationBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMa, final L listener) {
        this(measurements, commonAxisUsed, bias, initialMa, listener);
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
     * Gets x-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return x-coordinate of known accelerometer bias.
     */
    @Override
    public double getBiasX() {
        return biasX;
    }

    /**
     * Sets x-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasX x-coordinate of known accelerometer bias.
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
     * Gets y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return y-coordinate of known accelerometer bias.
     */
    @Override
    public double getBiasY() {
        return biasY;
    }

    /**
     * Sets y-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasY y-coordinate of known accelerometer bias.
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
     * Gets z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @return z-coordinate of known accelerometer bias.
     */
    @Override
    public double getBiasZ() {
        return biasZ;
    }

    /**
     * Sets z-coordinate of known accelerometer bias.
     * This is expressed in meters per squared second (m/s^2).
     *
     * @param biasZ z-coordinate of known accelerometer bias.
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
     * Gets x-coordinate of known accelerometer bias.
     *
     * @return x-coordinate of known accelerometer bias.
     */
    @Override
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets x-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(biasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets x-coordinate of known accelerometer bias.
     *
     * @param biasX x-coordinate of known accelerometer bias.
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
     * Gets y-coordinate of known accelerometer bias.
     *
     * @return y-coordinate of known accelerometer bias.
     */
    @Override
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets y-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(biasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets y-coordinate of known accelerometer bias.
     *
     * @param biasY y-coordinate of known accelerometer bias.
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
     * Gets z-coordinate of known accelerometer bias.
     *
     * @return z-coordinate of known accelerometer bias.
     */
    @Override
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets z-coordinate of known accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(biasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets z-coordinate of known accelerometer bias to be used to find a solution.
     *
     * @param biasZ z-coordinate of known accelerometer bias.
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
     * Sets known bias coordinates of accelerometer expressed in meters
     * per squared second (m/s^2).
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @param biasY y-coordinate of known accelerometer bias.
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(final double biasX, final double biasY, final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.biasX = biasX;
        this.biasY = biasY;
        this.biasZ = biasZ;
    }

    /**
     * Sets known bias coordinates of accelerometer.
     *
     * @param biasX x-coordinate of known accelerometer bias.
     * @param biasY y-coordinate of known accelerometer bias.
     * @param biasZ z-coordinate of known accelerometer bias.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setBiasCoordinates(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.biasX = convertAcceleration(biasX);
        this.biasY = convertAcceleration(biasY);
        this.biasZ = convertAcceleration(biasZ);
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
     * Gets known bias as an array.
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
     * Gets known bias as an array.
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
     * Sets known bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known bias to be set
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
     * Gets known bias as a column matrix.
     *
     * @return known bias as a column matrix.
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
     * Gets known bias as a column matrix.
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
     * Sets known bias as an array.
     *
     * @param bias bias to be set.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    @Override
    public void setBias(final Matrix bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (bias.getRows() != BodyKinematics.COMPONENTS || bias.getColumns() != 1) {
            throw new IllegalArgumentException();
        }

        biasX = bias.getElementAtIndex(0);
        biasY = bias.getElementAtIndex(1);
        biasZ = bias.getElementAtIndex(2);
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
    public void setMeasurements(final Collection<StandardDeviationBodyKinematics> measurements) throws LockedException {
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
     * Estimates accelerometer calibration parameters containing scale factors
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
     * Converts acceleration instance to meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value.
     */
    protected static double convertAcceleration(final Acceleration acceleration) {
        return convertAcceleration(acceleration.getValue().doubleValue(), acceleration.getUnit());
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

        // Notice that bias b is known, hence only terms in matrix M need to be estimated

        final var gradientEstimator = new GradientEstimator(this::evaluateGeneral);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                // cross coupling errors M
                return initialM.toArray();
            }

            @Override
            public double evaluate(final int i, final double[] point, final double[] params,
                                   final double[] derivatives) throws EvaluationException {

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

        final var m11 = result[0];
        final var m21 = result[1];
        final var m31 = result[2];

        final var m12 = result[3];
        final var m22 = result[4];
        final var m32 = result[5];

        final var m13 = result[6];
        final var m23 = result[7];
        final var m33 = result[8];

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

        setResult(crossCoupling);
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

        // Notice that bias b is known, hence only terms in matrix M need to be estimated

        final var gradientEstimator = new GradientEstimator(this::evaluateCommonAxis);

        final Matrix initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMa());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final double[] initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                // upper diagonal cross coupling errors M
                var k = 0;
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
            public double evaluate(final int i, final double[] point, final double[] params,
                                   final double[] derivatives) throws EvaluationException {

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

        final var m11 = result[0];

        final var m12 = result[1];
        final var m22 = result[2];

        final var m13 = result[3];
        final var m23 = result[4];
        final var m33 = result[5];

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

        setResult(crossCoupling);

        // taking into account that:
        // Ma = [sx  mxy  mxz] = [m11  m12  m13]
        //      [myx sy   myz]   [m21  m22  m23]
        //      [mzx mzy  sz ]   [m31  m32  m33]

        // propagate covariance so that all parameters are taken into account
        // in the order: sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy

        // We define a lineal function mapping original parameters for the
        // common axis case to the general case
        // [sx'] = [1  0  0  0  0  0][sx ]
        // [sy']   [0  0  1  0  0  0][mxy]
        // [sz']   [0  0  0  0  0  1][sy ]
        // [mxy']  [0  1  0  0  0  0][mxz]
        // [mxz']  [0  0  0  1  0  0][myz]
        // [myx']  [0  0  0  0  0  0][sz ]
        // [myz']  [0  0  0  0  1  0]
        // [mzx']  [0  0  0  0  0  0]
        // [mzy']  [0  0  0  0  0  0]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final var jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        jacobian.setElementAt(0, 0, 1.0);
        jacobian.setElementAt(1, 2, 1.0);
        jacobian.setElementAt(2, 5, 1.0);
        jacobian.setElementAt(3, 1, 1.0);
        jacobian.setElementAt(4, 3, 1.0);
        jacobian.setElementAt(6, 4, 1.0);
        // propagated covariance is J * Cov * J'
        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal cross-coupling matrix.
     */
    private void setResult(final Matrix m) {
        // Because:
        // M = I + Ma

        // Then:
        // Ma = M - I
        if (estimatedMa == null) {
            estimatedMa = m;
        } else {
            estimatedMa.copyFrom(m);
        }

        for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
            estimatedMa.setElementAt(i, i, estimatedMa.getElementAt(i, i) - 1.0);
        }

        // since only a constant term is subtracted, covariance is preserved
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
     *               Must have length 9.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneral(final double[] params) throws EvaluationException {
        final var m11 = params[0];
        final var m21 = params[1];
        final var m31 = params[2];

        final var m12 = params[3];
        final var m22 = params[4];
        final var m32 = params[5];

        final var m13 = params[6];
        final var m23 = params[7];
        final var m33 = params[8];

        return evaluate(m11, m21, m31, m12, m22, m32, m13, m23, m33);
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters when common z-axis is assumed.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the common z-axis case.
     *               Must have length 6.
     * @return estimated true specific force squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxis(final double[] params) throws EvaluationException {
        final var m11 = params[0];

        final var m12 = params[1];
        final var m22 = params[2];

        final var m13 = params[3];
        final var m23 = params[4];
        final var m33 = params[5];

        return evaluate(m11, 0.0, 0.0, m12, m22, 0.0, m13, m23, m33);
    }

    /**
     * Computes estimated true specific force squared norm using current measured
     * specific force and provided parameters.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
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
    private double evaluate(final double m11, final double m21, final double m31,
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
            if (ba == null) {
                ba = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            getBiasAsMatrix(ba);

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

            // b = M^-1*ba
            invM.multiply(ba, b);

            invM.multiply(fmeas, ftrue);
            ftrue.subtract(b);

            final var norm = Utils.normF(ftrue);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }
}
