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
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Estimates accelerometer biases, cross couplings and scaling factors.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
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
public class KnownFrameAccelerometerNonLinearLeastSquaresCalibrator implements
        KnownFrameAccelerometerCalibrator<StandardDeviationFrameBodyKinematics,
                KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener>, AccelerometerNonLinearCalibrator,
        UnknownBiasNonLinearAccelerometerCalibrator, AccelerometerCalibrationSource,
        AccelerometerBiasUncertaintySource, UnorderedStandardDeviationFrameBodyKinematicsAccelerometerCalibrator {

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
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 12;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter fitter = new LevenbergMarquardtMultiVariateFitter();

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
     * Contains a collection of body kinematics measurements taken at different
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
    private Collection<StandardDeviationFrameBodyKinematics> measurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener;

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
     * Indicates whether calibrator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBiasX initial x-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasY initial y-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param initialBiasZ initial z-coordinate of accelerometer bias to be used
     *                     to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBiasX   initial x-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasY   initial y-coordinate of accelerometer bias to be used
     *                       to find a solution.
     * @param initialBiasZ   initial z-coordinate of accelerometer bias to be used
     *                       to find a solution.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX,
            final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz,
                    initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialBiasX, final double initialBiasY,
            final double initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double initialBiasX, final double initialBiasY, final double initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ);
        try {
            setInitialScalingFactorsAndCrossCouplingErrors(initialSx, initialSy, initialSz,
                    initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Acceleration initialBiasX, final Acceleration initialBiasY,
            final Acceleration initialBiasZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialBiasX, initialBiasY, initialBiasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(final double[] initialBias) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final double[] initialBias, final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final double[] initialBias) {
        this(initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial accelerometer bias to be used to find a solution.
     *                     This must have length 3 and is expressed in meters per
     *                     squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final double[] initialBias,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialBias) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialBias,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double[] initialBias) {
        this(commonAxisUsed, initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial accelerometer bias to be used to find a solution.
     *                       This must have length 3 and is expressed in meters per
     *                       squared second (m/s^2).
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] initialBias, final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialBias initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(final Matrix initialBias) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final Matrix initialBias) {
        this(initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final Matrix initialBias,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias) {
        this(commonAxisUsed, initialBias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix initialBias, final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(final Matrix initialBias, final Matrix initialMa) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Matrix initialBias, final Matrix initialMa,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa) {
        this(initialBias, initialMa);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param initialBias  initial bias to find a solution.
     * @param initialMa    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix initialBias, final Matrix initialMa,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
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
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialBias, initialMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa) {
        this(commonAxisUsed, initialBias, initialMa);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param initialBias    initial bias to find a solution.
     * @param initialMa      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix initialBias, final Matrix initialMa,
            final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialBias, initialMa);
        this.listener = listener;
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
    public void setInitialBias(
            final double initialBiasX, final double initialBiasY, final double initialBiasZ) throws LockedException {
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
    public void setInitialBias(
            final Acceleration initialBiasX, final Acceleration initialBiasY, final Acceleration initialBiasZ)
            throws LockedException {
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
     * Gets a collection of body kinematics measurements taken at different
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
     *
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<StandardDeviationFrameBodyKinematics> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at different
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
     *
     * @param measurements collection of body kinematics measurements taken at different
     *                     frames (positions, orientations and velocities).
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setMeasurements(
            final Collection<? extends StandardDeviationFrameBodyKinematics> measurements) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        //noinspection unchecked
        this.measurements = (Collection<StandardDeviationFrameBodyKinematics>) measurements;
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
    @Override
    public KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(final KnownFrameAccelerometerNonLinearLeastSquaresCalibratorListener listener)
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
                listener.onCalibrateStart(this);
            }

            if (commonAxisUsed) {
                calibrateCommonAxis();
            } else {
                calibrateGeneral();
            }

            if (listener != null) {
                listener.onCalibrateEnd(this);
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
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[0], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[1], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null
                ? new Acceleration(estimatedBiases[2], AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
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
        return estimatedBiases != null
                ? new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                estimatedBiases[0], estimatedBiases[1], estimatedBiases[2])
                : null;
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
            result.setValueCoordinatesAndUnit(
                    estimatedBiases[0], estimatedBiases[1], estimatedBiases[2],
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
        return estimatedCovariance != null
                ? new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                getEstimatedBiasFxStandardDeviation(),
                getEstimatedBiasFyStandardDeviation(),
                getEstimatedBiasFzStandardDeviation())
                : null;
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
        return estimatedCovariance != null
                ? (getEstimatedBiasFxStandardDeviation() + getEstimatedBiasFyStandardDeviation()
                + getEstimatedBiasFzStandardDeviation()) / 3.0 : null;
    }

    /**
     * Gets average of estimated standard deviation of accelerometer bias coordinates.
     *
     * @return average of estimated standard deviation of accelerometer bias coordinates or null.
     */
    public Acceleration getEstimatedBiasStandardDeviationAverageAsAcceleration() {
        return estimatedCovariance != null
                ? new Acceleration(getEstimatedBiasStandardDeviationAverage(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND)
                : null;
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        // [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        // [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        // [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        // where myx = mzx = mzy = 0

        // Hence:
        // [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        // [fmeasy] = [by]     [0  1   0]   [0     sy  myz]    [ftruey]
        // [fmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [ftruez]

        // [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        // [fmeasy]   [by]     [0      1+sy    myz ][ftruey]
        // [fmeasz]   [bz]     [0      0       1+sz][ftruez]

        // fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + (1+sy) * ftruey + myz * ftruez
        // fmeasz = bz + (1+sz) * ftruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz
        // Reordering:
        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + ftruez + sz * ftruez

        // fmeasx - ftruex = bx + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy - ftruey = by + sy * ftruey + myz * ftruez
        // fmeasz - ftruez = bz + sz * ftruez

        // [1   0   0   ftruex  0       0       ftruey  ftruez  0     ][bx ] = [fmeasx - ftruex]
        // [0   1   0   0       ftruey  0       0       0       ftruez][by ]   [fmeasy - ftruey]
        // [0   0   1   0       0       ftruez  0       0       0     ][bz ]   [fmeasz - ftruez]
        //                                                             [sx ]
        //                                                             [sy ]
        //                                                             [sz ]
        //                                                             [mxy]
        //                                                             [mxz]
        //                                                             [myz]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured specific force
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                initial[0] = initialBiasX;
                initial[1] = initialBiasY;
                initial[2] = initialBiasZ;

                initial[3] = initialSx;
                initial[4] = initialSy;
                initial[5] = initialSz;

                initial[6] = initialMxy;
                initial[7] = initialMxz;
                initial[8] = initialMyz;

                return initial;
            }

            @Override
            public void evaluate(final int i, final double[] point, final double[] result, final double[] params,
                                 final Matrix jacobian) {
                // We know that:
                // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
                // fmeasy = by + ftruey + sy * ftruey + myz * ftruez
                // fmeasz = bz + ftruez + sz * ftruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myz

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = ftruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = ftruey
                // d(fmeasx)/d(mxz) = ftruez
                // d(fmeasx)/d(myz) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = ftruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myz) = ftruez

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = ftruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myz) = 0.0

                final var bx = params[0];
                final var by = params[1];
                final var bz = params[2];

                final var sx = params[3];
                final var sy = params[4];
                final var sz = params[5];

                final var mxy = params[6];
                final var mxz = params[7];
                final var myz = params[8];

                final var ftruex = point[0];
                final var ftruey = point[1];
                final var ftruez = point[2];

                result[0] = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez;
                result[1] = by + ftruey + sy * ftruey + myz * ftruez;
                result[2] = bz + ftruez + sz * ftruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, ftruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, ftruey);
                jacobian.setElementAt(0, 7, ftruez);
                jacobian.setElementAt(0, 8, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, ftruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, ftruez);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, ftruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var bx = result[0];
        final var by = result[1];
        final var bz = result[2];

        final var sx = result[3];
        final var sy = result[4];
        final var sz = result[5];

        final var mxy = result[6];
        final var mxz = result[7];
        final var myz = result[8];

        if (estimatedBiases == null) {
            estimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        estimatedBiases[0] = bx;
        estimatedBiases[1] = by;
        estimatedBiases[2] = bz;

        if (estimatedMa == null) {
            estimatedMa = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedMa.initialize(0.0);
        }

        estimatedMa.setElementAt(0, 0, sx);

        estimatedMa.setElementAt(0, 1, mxy);
        estimatedMa.setElementAt(1, 1, sy);

        estimatedMa.setElementAt(0, 2, mxz);
        estimatedMa.setElementAt(1, 2, myz);
        estimatedMa.setElementAt(2, 2, sz);

        estimatedCovariance = fitter.getCovar();

        // propagate covariance matrix so that all parameters are taken into
        // account in the order: bx, by, bz, sx, sy, sz, mxy, mxz, myx,
        // myz, mzx, mzy.

        // We define a lineal function mapping original parameters for the common
        // axis case to the general case
        // [bx'] = [1  0  0  0  0  0  0  0  0][bx]
        // [by']   [0  1  0  0  0  0  0  0  0][by]
        // [bz']   [0  0  1  0  0  0  0  0  0][bz]
        // [sx']   [0  0  0  1  0  0  0  0  0][sx]
        // [sy']   [0  0  0  0  1  0  0  0  0][sy]
        // [sz']   [0  0  0  0  0  1  0  0  0][sz]
        // [mxy']  [0  0  0  0  0  0  1  0  0][mxy]
        // [mxz']  [0  0  0  0  0  0  0  1  0][mxz]
        // [myx']  [0  0  0  0  0  0  0  0  0][myz]
        // [myz']  [0  0  0  0  0  0  0  0  1]
        // [mzx']  [0  0  0  0  0  0  0  0  0]
        // [mzy']  [0  0  0  0  0  0  0  0  0]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final var jacobian = Matrix.identity(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        jacobian.setElementAt(8, 8, 0.0);
        jacobian.setElementAt(9, 8, 1.0);
        // propagated covariance is J * Cov * J'
        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {
        // The accelerometer model is:
        // fmeas = ba + (I + Ma) * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // fmeas = ba + (I + Ma) * ftrue

        // Hence:
        // [fmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [ftruex]
        // [fmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        // [fmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

        // [fmeasx] = [bx] +   [1+sx   mxy     mxz ][ftruex]
        // [fmeasy]   [by]     [myx    1+sy    myz ][ftruey]
        // [fmeasz]   [bz]     [mzx    mzy     1+sz][ftruez]

        // fmeasx = bx + (1+sx) * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + myx * ftruex + (1+sy) * ftruey + myz * ftruez
        // fmeasz = bz + mzx * ftruex + mzy * ftruey + (1+sz) * ftruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        // fmeasx - ftruex = bx + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy - ftruey = by + myx * ftruex + sy * ftruey + myz * ftruez
        // fmeasz - ftruez = bz + mzx * ftruex + mzy * ftruey + sz * ftruez

        // [1   0   0   ftruex  0       0       ftruey  ftruez  0       0       0       0     ][bx ] = [fmeasx - ftruex]
        // [0   1   0   0       ftruey  0       0       0       ftruex  ftruez  0       0     ][by ]   [fmeasy - ftruey]
        // [0   0   1   0       0       ftruez  0       0       0       0       ftruex  ftruey][bz ]   [fmeasz - ftruez]
        //                                                                                     [sx ]
        //                                                                                     [sy ]
        //                                                                                     [sz ]
        //                                                                                     [mxy]
        //                                                                                     [mxz]
        //                                                                                     [myx]
        //                                                                                     [myz]
        //                                                                                     [mzx]
        //                                                                                     [mzy]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true specific force coordinates
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured specific force
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS];

                initial[0] = initialBiasX;
                initial[1] = initialBiasY;
                initial[2] = initialBiasZ;

                initial[3] = initialSx;
                initial[4] = initialSy;
                initial[5] = initialSz;

                initial[6] = initialMxy;
                initial[7] = initialMxz;
                initial[8] = initialMyx;
                initial[9] = initialMyz;
                initial[10] = initialMzx;
                initial[11] = initialMzy;

                return initial;
            }

            @Override
            public void evaluate(final int i, final double[] point, final double[] result, final double[] params,
                                 final Matrix jacobian) {
                // We know that:
                // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
                // fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
                // fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myx, myz, mzx and mzy is:

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = ftruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = ftruey
                // d(fmeasx)/d(mxz) = ftruez
                // d(fmeasx)/d(myx) = 0.0
                // d(fmeasx)/d(myz) = 0.0
                // d(fmeasx)/d(mzx) = 0.0
                // d(fmeasx)/d(mzy) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = ftruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myx) = ftruex
                // d(fmeasy)/d(myz) = ftruez
                // d(fmeasy)/d(mzx) = 0.0
                // d(fmeasy)/d(mzy) = 0.0

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = ftruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myx) = 0.0
                // d(fmeasz)/d(myz) = 0.0
                // d(fmeasz)/d(mzx) = ftruex
                // d(fmeasz)/d(mzy) = ftruey

                final var bx = params[0];
                final var by = params[1];
                final var bz = params[2];

                final var sx = params[3];
                final var sy = params[4];
                final var sz = params[5];

                final var mxy = params[6];
                final var mxz = params[7];
                final var myx = params[8];
                final var myz = params[9];
                final var mzx = params[10];
                final var mzy = params[11];

                final var ftruex = point[0];
                final var ftruey = point[1];
                final var ftruez = point[2];

                result[0] = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez;
                result[1] = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez;
                result[2] = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, ftruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, ftruey);
                jacobian.setElementAt(0, 7, ftruez);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, 0.0);
                jacobian.setElementAt(0, 10, 0.0);
                jacobian.setElementAt(0, 11, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, ftruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, ftruex);
                jacobian.setElementAt(1, 9, ftruez);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, ftruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, ftruex);
                jacobian.setElementAt(2, 11, ftruey);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var bx = result[0];
        final var by = result[1];
        final var bz = result[2];

        final var sx = result[3];
        final var sy = result[4];
        final var sz = result[5];

        final var mxy = result[6];
        final var mxz = result[7];
        final var myx = result[8];
        final var myz = result[9];
        final var mzx = result[10];
        final var mzy = result[11];

        if (estimatedBiases == null) {
            estimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        estimatedBiases[0] = bx;
        estimatedBiases[1] = by;
        estimatedBiases[2] = bz;

        if (estimatedMa == null) {
            estimatedMa = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedMa.initialize(0.0);
        }

        estimatedMa.setElementAt(0, 0, sx);
        estimatedMa.setElementAt(1, 0, myx);
        estimatedMa.setElementAt(2, 0, mzx);

        estimatedMa.setElementAt(0, 1, mxy);
        estimatedMa.setElementAt(1, 1, sy);
        estimatedMa.setElementAt(2, 1, mzy);

        estimatedMa.setElementAt(0, 2, mxz);
        estimatedMa.setElementAt(1, 2, myz);
        estimatedMa.setElementAt(2, 2, sz);

        estimatedCovariance = fitter.getCovar();
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     */
    private void setInputData() throws WrongSizeException {
        // set input data using:
        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        final var expectedKinematics = new BodyKinematics();

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final var y = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final var specificForceStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredKinematics = measurement.getKinematics();
            final var ecefFrame = measurement.getFrame();
            final var previousEcefFrame = measurement.getPreviousFrame();
            final var timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame, previousEcefFrame, expectedKinematics);

            final var fMeasX = measuredKinematics.getFx();
            final var fMeasY = measuredKinematics.getFy();
            final var fMeasZ = measuredKinematics.getFz();

            final var fTrueX = expectedKinematics.getFx();
            final var fTrueY = expectedKinematics.getFy();
            final var fTrueZ = expectedKinematics.getFz();

            x.setElementAt(i, 0, fTrueX);
            x.setElementAt(i, 1, fTrueY);
            x.setElementAt(i, 2, fTrueZ);

            y.setElementAt(i, 0, fMeasX);
            y.setElementAt(i, 1, fMeasY);
            y.setElementAt(i, 2, fMeasZ);

            specificForceStandardDeviations[i] = measurement.getSpecificForceStandardDeviation();
            i++;
        }

        fitter.setInputData(x, y, specificForceStandardDeviations);
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
