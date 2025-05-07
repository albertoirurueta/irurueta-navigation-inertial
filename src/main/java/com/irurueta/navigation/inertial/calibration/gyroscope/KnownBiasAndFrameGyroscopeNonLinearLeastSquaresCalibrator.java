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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;

/**
 * Estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 6 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 6 different positions, orientations and velocities
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
public class KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator implements
        KnownBiasAndFrameGyroscopeCalibrator<StandardDeviationFrameBodyKinematics,
                KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener>, GyroscopeNonLinearCalibrator,
        UnorderedStandardDeviationFrameBodyKinematicsGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 6;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 15;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 18;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter fitter = new LevenbergMarquardtMultiVariateFitter();

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
     * Contains a collections of body kinematics measurements taken at different
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
    private KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener;

    /**
     * Known x coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double biasX;

    /**
     * Known y coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double biasY;

    /**
     * Known z coordinate of gyroscope bias expressed in radians per second (rad/s).
     */
    private double biasZ;

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
     * Indicates whether calibrator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator() {
        try {
            initialGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this();
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements) {
        this();
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per second
     *              (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        this();
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per second
     *                 (rad/s).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this();
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY,
            final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        this(commonAxisUsed, biasX, biasY, biasZ);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param biasY     known y coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param biasZ     known z coordinate of gyroscope bias expressed in radians per second
     *                  (rad/s).
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX     known x coordinate of gyroscope bias.
     * @param biasY     known y coordinate of gyroscope bias.
     * @param biasZ     known z coordinate of gyroscope bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
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
     * @param biasX     known x coordinate of gyroscope bias.
     * @param biasY     known y coordinate of gyroscope bias.
     * @param biasZ     known z coordinate of gyroscope bias.
     * @param initialSx initial x scaling factor.
     * @param initialSy initial y scaling factor.
     * @param initialSz initial z scaling factor.
     * @param listener  listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param initialSx    initial x scaling factor.
     * @param initialSy    initial y scaling factor.
     * @param initialSz    initial z scaling factor.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
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
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param initialSx      initial x scaling factor.
     * @param initialSy      initial y scaling factor.
     * @param initialSz      initial z scaling factor.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
     * @param biasY      known y coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
     * @param biasZ      known z coordinate of gyroscope bias expressed in radians per second
     *                   (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ);
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
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per second
     *                     (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double biasX, final double biasY, final double biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX      known x coordinate of gyroscope bias.
     * @param biasY      known y coordinate of gyroscope bias.
     * @param biasZ      known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ);
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
     * @param biasX      known x coordinate of gyroscope bias.
     * @param biasY      known y coordinate of gyroscope bias.
     * @param biasZ      known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
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
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
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
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per second
     *                       (rad/s).
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
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias expressed in radians per second (rad/s).
     *             This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(final double[] bias) {
        this();
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias expressed in radians per second (rad/s).
     *                 This must have length 3.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final double[] bias, final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias expressed in radians per second (rad/s).
     *                     This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final double[] bias) {
        this(bias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias expressed in radians per second (rad/s).
     *                     This must have length 3.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] bias) {
        this(bias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known gyroscope bias expressed in radians per second (rad/s).
     *                       This must have length 3.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias array does not have length 3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final double[] bias, final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias known bias expressed in radians per second (rad/s). This must
     *             be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(final Matrix bias) {
        this();
        try {
            setBias(bias);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias     known bias expressed in radians per second (rad/s). This must
     *                 be 3x1.
     * @param listener listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias) {
        this(bias);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(final boolean commonAxisUsed, final Matrix bias) {
        this(bias);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix bias) {
        this(commonAxisUsed, bias);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided bias matrix is not 3x1.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(final Matrix bias, final Matrix initialMg) {
        this(bias);
        try {
            setInitialMg(initialMg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, initialMg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg) {
        this(bias, initialMg);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, initialMg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg) {
        this(bias, initialMg);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias, initialMg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix bias, final Matrix initialMg) {
        this(commonAxisUsed, bias, initialMg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1 or
     *                                  scaling and coupling error matrix is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias, initialMg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param initialGg initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg, final Matrix initialGg) {
        this(bias, initialMg);
        try {
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param bias      known bias expressed in radians per second (rad/s). This must
     *                  be 3x1.
     * @param initialMg initial scale factors and cross coupling errors matrix.
     * @param initialGg initial G-dependent cross biases.
     * @param listener  listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Matrix bias, final Matrix initialMg, final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(bias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg, final Matrix initialGg) {
        this(bias, initialMg, initialGg);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known bias expressed in radians per second (rad/s). This must
     *                     be 3x1.
     * @param initialMg    initial scale factors and cross coupling errors matrix.
     * @param initialGg    initial G-dependent cross biases.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final Matrix initialMg, final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, bias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg, final Matrix initialGg) {
        this(bias, initialMg, initialGg);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg, final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, bias, initialMg, initialGg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final Matrix bias, final Matrix initialMg, final Matrix initialGg) {
        this(commonAxisUsed, bias, initialMg, initialGg);
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
     * @param bias           known bias expressed in radians per second (rad/s). This must
     *                       be 3x1.
     * @param initialMg      initial scale factors and cross coupling errors matrix.
     * @param initialGg      initial G-dependent cross biases.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if either provided bias matrix is not 3x1,
     *                                  scaling and coupling error matrix is not 3x3
     *                                  or g-dependant cross biases is not 3x3.
     */
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final Matrix bias, final Matrix initialMg, final Matrix initialGg,
            final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, bias, initialMg, initialGg);
        this.listener = listener;
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
            final double initialMxy, final double initialMxz,
            final double initialMyx, final double initialMyz,
            final double initialMzx, final double initialMzy) throws LockedException {
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
     * Gets a collection of body kinematics measurements taken at different
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
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<StandardDeviationFrameBodyKinematics> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a collection of body kinematics measurements taken at different
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
        return false;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement/sequence or not.
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
     * When enabled, this eliminates 3 variables from Mg matrix.
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
     * When enabled, this eliminates 3 variables from Mg matrix.
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
     * Gets listener to handle events raised by this calibrator.
     *
     * @return listener to handle events raised by this calibrator.
     */
    @Override
    public KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(final KnownBiasAndFrameGyroscopeNonLinearLeastSquaresCalibratorListener listener)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public double getBiasX() {
        return biasX;
    }

    /**
     * Sets known x coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
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
     * Gets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public double getBiasY() {
        return biasY;
    }

    /**
     * Sets known y coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y coordinate of gyroscope bias.
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
     * Gets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public double getBiasZ() {
        return biasZ;
    }

    /**
     * Sets known z coordinate of gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z coordinate of gyroscope bias.
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
     * Gets known x coordinate of gyroscope bias.
     *
     * @return x coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedX() {
        return new AngularSpeed(biasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known x coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(biasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known x coordinate of gyroscope bias.
     *
     * @param biasX x coordinate of gyroscope bias.
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
     * Gets known y coordinate of gyroscope bias.
     *
     * @return y coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedY() {
        return new AngularSpeed(biasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known y coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(biasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known y coordinate of gyroscope bias.
     *
     * @param biasY y coordinate of gyroscope bias.
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
     * Gets known z coordinate of gyroscope bias.
     *
     * @return z coordinate of gyroscope bias.
     */
    @Override
    public AngularSpeed getBiasAngularSpeedZ() {
        return new AngularSpeed(biasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets known z coordinate of gyroscope bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(biasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets known z coordinate of gyroscope bias.
     *
     * @param biasZ z coordinate of gyroscope bias.
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
     * Sets known gyroscope bias coordinates expressed in radians per second
     * (rad/s).
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
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
     * Sets known gyroscope bias coordinates.
     *
     * @param biasX x coordinate of gyroscope bias.
     * @param biasY y coordinate of gyroscope bias.
     * @param biasZ z coordinate of gyroscope bias.
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
     * Gets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinate of known bias.
     */
    @Override
    public double[] getBias() {
        final var result = new double[BodyKinematics.COMPONENTS];
        getBias(result);
        return result;
    }

    /**
     * Gets known gyroscope bias as an array.
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
     * Sets known gyroscope bias as an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param bias known gyroscope bias.
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
     * Gets known gyroscope bias as a column matrix.
     *
     * @return known gyroscope bias as a column matrix.
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
     * Gets known gyroscope bias as a column matrix.
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
     * Sets known gyroscope bias as a column matrix.
     *
     * @param bias gyroscope bias to be set.
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
     * Gets minimum number of required measurements.
     *
     * @return minimum number of required measurements.
     */
    @Override
    public int getMinimumRequiredMeasurementsOrSequences() {
        return MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether calibrator is ready to start the calibration.
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
     * Estimates gyroscope calibration parameters containing bias, scale factors,
     * cross-coupling errors and g-dependant cross biases.
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
     * @return estimated gyroscope scale factors and cross coupling errors.
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
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    @Override
    public Matrix getEstimatedGg() {
        return estimatedGg;
    }

    /**
     * Gets estimated covariance matrix for estimated position.
     * Diagonal elements of the matrix contains variance for the following
     * parameters (following indicated order): sx, sy, sz, mxy, mxz, myx,
     * myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32, gg13, gg23, gg33.
     *
     * @return estimated covariance matrix for estimated position.
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException                         if there are numerical errors.
     * @throws FittingException                         if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException if fitter is not ready.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException {

        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // where myx = mzx = mzy = 0

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [0     sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [0     0      sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0     1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0     0      1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23, g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruez  0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                         [mxy]
        //                                                                                                                         [mxz]
        //                                                                                                                         [myz]
        //                                                                                                                         [g11]
        //                                                                                                                         [g12]
        //                                                                                                                         [g13]
        //                                                                                                                         [g21]
        //                                                                                                                         [g22]
        //                                                                                                                         [g23]
        //                                                                                                                         [g31]
        //                                                                                                                         [g32]
        //                                                                                                                         [g33]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured angular rate
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                initial[0] = initialSx;
                initial[1] = initialSy;
                initial[2] = initialSz;

                initial[3] = initialMxy;
                initial[4] = initialMxz;
                initial[5] = initialMyz;

                final var buffer = initialGg.getBuffer();
                final var num = buffer.length;
                System.arraycopy(buffer, 0, initial, 6, num);

                return initial;
            }

            @Override
            public void evaluate(
                    final int i, final double[] point, final double[] result, final double[] params,
                    final Matrix jacobian) {
                // We know that:
                // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
                // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
                // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

                // Hence, the derivatives respect the parameters
                // sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23,
                // g31, g32, and g33 is:

                // d(Ωmeasx)/d(sx) = Ωtruex
                // d(Ωmeasx)/d(sy) = 0.0
                // d(Ωmeasx)/d(sz) = 0.0
                // d(Ωmeasx)/d(mxy) = Ωtruey
                // d(Ωmeasx)/d(mxz) = Ωtruez
                // d(Ωmeasx)/d(myz) = 0.0
                // d(Ωmeasx)/d(g11) = ftruex
                // d(Ωmeasx)/d(g12) = ftruey
                // d(Ωmeasx)/d(g13) = ftruez
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasy)/d(sx) = 0.0
                // d(Ωmeasy)/d(sy) = Ωtruey
                // d(Ωmeasy)/d(sz) = 0.0
                // d(Ωmeasy)/d(mxy) = 0.0
                // d(Ωmeasy)/d(mxz) = 0.0
                // d(Ωmeasy)/d(myz) = Ωtruez
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = ftruex
                // d(Ωmeasx)/d(g22) = ftruey
                // d(Ωmeasx)/d(g23) = ftruez
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasz)/d(sx) = 0.0
                // d(Ωmeasz)/d(sy) = 0.0
                // d(Ωmeasz)/d(sz) = Ωtruez
                // d(Ωmeasz)/d(mxy) = 0.0
                // d(Ωmeasz)/d(mxz) = 0.0
                // d(Ωmeasz)/d(myz) = 0.0
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = ftruex
                // d(Ωmeasx)/d(g32) = ftruey
                // d(Ωmeasx)/d(g33) = ftruez

                final var sx = params[0];
                final var sy = params[1];
                final var sz = params[2];

                final var mxy = params[3];
                final var mxz = params[4];
                final var myz = params[5];

                final var g11 = params[6];
                final var g21 = params[7];
                final var g31 = params[8];
                final var g12 = params[9];
                final var g22 = params[10];
                final var g32 = params[11];
                final var g13 = params[12];
                final var g23 = params[13];
                final var g33 = params[14];

                final var omegatruex = point[0];
                final var omegatruey = point[1];
                final var omegatruez = point[2];

                final var ftruex = point[3];
                final var ftruey = point[4];
                final var ftruez = point[5];

                result[0] = biasX + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                        + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                result[1] = biasY + omegatruey + sy * omegatruey + myz * omegatruez
                        + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                result[2] = biasZ + omegatruez + sz * omegatruez
                        + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                jacobian.setElementAt(0, 0, omegatruex);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, omegatruey);
                jacobian.setElementAt(0, 4, omegatruez);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, ftruex);
                jacobian.setElementAt(0, 7, ftruey);
                jacobian.setElementAt(0, 8, ftruez);
                jacobian.setElementAt(0, 9, 0.0);
                jacobian.setElementAt(0, 10, 0.0);
                jacobian.setElementAt(0, 11, 0.0);
                jacobian.setElementAt(0, 12, 0.0);
                jacobian.setElementAt(0, 13, 0.0);
                jacobian.setElementAt(0, 14, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, omegatruey);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, 0.0);
                jacobian.setElementAt(1, 5, omegatruez);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, 0.0);
                jacobian.setElementAt(1, 9, ftruex);
                jacobian.setElementAt(1, 10, ftruey);
                jacobian.setElementAt(1, 11, ftruez);
                jacobian.setElementAt(1, 12, 0.0);
                jacobian.setElementAt(1, 13, 0.0);
                jacobian.setElementAt(1, 14, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, omegatruez);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, 0.0);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, 0.0);
                jacobian.setElementAt(2, 11, 0.0);
                jacobian.setElementAt(2, 12, ftruex);
                jacobian.setElementAt(2, 13, ftruey);
                jacobian.setElementAt(2, 14, ftruez);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var sx = result[0];
        final var sy = result[1];
        final var sz = result[2];

        final var mxy = result[3];
        final var mxz = result[4];
        final var myz = result[5];

        final var g11 = result[6];
        final var g21 = result[7];
        final var g31 = result[8];
        final var g12 = result[9];
        final var g22 = result[10];
        final var g32 = result[11];
        final var g13 = result[12];
        final var g23 = result[13];
        final var g33 = result[14];

        if (estimatedMg == null) {
            estimatedMg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedMg.initialize(0.0);
        }

        estimatedMg.setElementAt(0, 0, sx);

        estimatedMg.setElementAt(0, 1, mxy);
        estimatedMg.setElementAt(1, 1, sy);

        estimatedMg.setElementAt(0, 2, mxz);
        estimatedMg.setElementAt(1, 2, myz);
        estimatedMg.setElementAt(2, 2, sz);

        if (estimatedGg == null) {
            estimatedGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedGg.initialize(0.0);
        }

        estimatedGg.setElementAtIndex(0, g11);
        estimatedGg.setElementAtIndex(1, g21);
        estimatedGg.setElementAtIndex(2, g31);
        estimatedGg.setElementAtIndex(3, g12);
        estimatedGg.setElementAtIndex(4, g22);
        estimatedGg.setElementAtIndex(5, g32);
        estimatedGg.setElementAtIndex(6, g13);
        estimatedGg.setElementAtIndex(7, g23);
        estimatedGg.setElementAtIndex(8, g33);

        estimatedCovariance = fitter.getCovar();

        // propagate covariance matrix so that all parameters are taken into
        // account in the order: sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
        // g11, g21, g31, g12, g22, g32, g13, g23, g33

        // We define a lineal function mapping original parameters for the common
        // axis case to the general case
        // [sx'] =   [1  0  0  0  0  0  0  0  0  0  0  0  0  0  0][sx]
        // [sy']     [0  1  0  0  0  0  0  0  0  0  0  0  0  0  0][sy]
        // [sz']     [0  0  1  0  0  0  0  0  0  0  0  0  0  0  0][sz]
        // [mxy']    [0  0  0  1  0  0  0  0  0  0  0  0  0  0  0][mxy]
        // [mxz']    [0  0  0  0  1  0  0  0  0  0  0  0  0  0  0][mxz]
        // [myx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][myz]
        // [myz']    [0  0  0  0  0  1  0  0  0  0  0  0  0  0  0][g11]
        // [mzx']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g21]
        // [mzy']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0][g31]
        // [g11']    [0  0  0  0  0  0  1  0  0  0  0  0  0  0  0][g12]
        // [g21']    [0  0  0  0  0  0  0  1  0  0  0  0  0  0  0][g22]
        // [g31']    [0  0  0  0  0  0  0  0  1  0  0  0  0  0  0][g32]
        // [g12']    [0  0  0  0  0  0  0  0  0  1  0  0  0  0  0][g13]
        // [g22']    [0  0  0  0  0  0  0  0  0  0  1  0  0  0  0][g23]
        // [g32']    [0  0  0  0  0  0  0  0  0  0  0  1  0  0  0][g33]
        // [g13']    [0  0  0  0  0  0  0  0  0  0  0  0  1  0  0]
        // [g23']    [0  0  0  0  0  0  0  0  0  0  0  0  0  1  0]
        // [g33']    [0  0  0  0  0  0  0  0  0  0  0  0  0  0  1]

        // As defined in com.irurueta.statistics.MultivariateNormalDist,
        // if we consider the jacobian of the lineal application the matrix shown
        // above, then covariance can be propagated as follows
        final var jacobian = new Matrix(GENERAL_UNKNOWNS, COMMON_Z_AXIS_UNKNOWNS);
        for (var i = 0; i < 5; i++) {
            jacobian.setElementAt(i, i, 1.0);
        }
        jacobian.setElementAt(6, 5, 1.0);

        for (int j = 6, i = 9; j < 15; j++, i++) {
            jacobian.setElementAt(i, j, 1.0);
        }

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

        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

        // [Ωmeasx] = [bx] + ( [1+sx  mxy    mxz ]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [myx   1+sy   myz ]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [mzx   mzy    1+sz]  [Ωtruez]   [g31   g32   g33][ftruez]

        // Ωmeasx = bx + (1+sx) * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + (1+sy) * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + (1+sz) * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Where the unknowns are: sx, sy, sz, mxy mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
        // g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex - bx = sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey - by = myx * Ωtruex + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez - bz = mzx * Ωtruex + mzy * Ωtruey + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [Ωtruex  0       0       Ωtruey  Ωtruez  0       0       0       0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][sx ] =  [Ωmeasx - Ωtruex - bx]
        // [0       Ωtruey  0       0       0       Ωtruex  Ωtruez  0       0       0       0       0       ftruex  ftruey  ftruez  0       0       0     ][sy ]    [Ωmeasy - Ωtruey - by]
        // [0       0       Ωtruez  0       0       0       0       Ωtruex  Ωtruey  0       0       0       0       0       0       ftruex  ftruey  ftruez][sz ]    [Ωmeasz - Ωtruez - bz]
        //                                                                                                                                                 [mxy]
        //                                                                                                                                                 [mxz]
        //                                                                                                                                                 [myx]
        //                                                                                                                                                 [myz]
        //                                                                                                                                                 [mzx]
        //                                                                                                                                                 [mzy]
        //                                                                                                                                                 [g11]
        //                                                                                                                                                 [g12]
        //                                                                                                                                                 [g13]
        //                                                                                                                                                 [g21]
        //                                                                                                                                                 [g22]
        //                                                                                                                                                 [g23]
        //                                                                                                                                                 [g31]
        //                                                                                                                                                 [g32]
        //                                                                                                                                                 [g33]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true angular rate + specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured angular rate
                return BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS];

                initial[0] = initialSx;
                initial[1] = initialSy;
                initial[2] = initialSz;

                initial[3] = initialMxy;
                initial[4] = initialMxz;
                initial[5] = initialMyx;
                initial[6] = initialMyz;
                initial[7] = initialMzx;
                initial[8] = initialMzy;

                final var buffer = initialGg.getBuffer();
                final var num = buffer.length;
                System.arraycopy(buffer, 0, initial, 9, num);

                return initial;
            }

            @Override
            public void evaluate(final int i, final double[] point, final double[] result, final double[] params,
                                 final Matrix jacobian) {
                // We know that:
                // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
                // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
                // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

                // Hence, the derivatives respect the parameters sx, sy,
                // sz, mxy, mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
                // g31, g32 and g33 is:

                // d(Ωmeasx)/d(sx) = Ωtruex
                // d(Ωmeasx)/d(sy) = 0.0
                // d(Ωmeasx)/d(sz) = 0.0
                // d(Ωmeasx)/d(mxy) = Ωtruey
                // d(Ωmeasx)/d(mxz) = Ωtruez
                // d(Ωmeasx)/d(myx) = 0.0
                // d(Ωmeasx)/d(myz) = 0.0
                // d(Ωmeasx)/d(mzx) = 0.0
                // d(Ωmeasx)/d(mzy) = 0.0
                // d(Ωmeasx)/d(g11) = ftruex
                // d(Ωmeasx)/d(g12) = ftruey
                // d(Ωmeasx)/d(g13) = ftruez
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasy)/d(sx) = 0.0
                // d(Ωmeasy)/d(sy) = Ωtruey
                // d(Ωmeasy)/d(sz) = 0.0
                // d(Ωmeasy)/d(mxy) = 0.0
                // d(Ωmeasy)/d(mxz) = 0.0
                // d(Ωmeasy)/d(myx) = Ωtruex
                // d(Ωmeasy)/d(myz) = Ωtruez
                // d(Ωmeasy)/d(mzx) = 0.0
                // d(Ωmeasy)/d(mzy) = 0.0
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = ftruex
                // d(Ωmeasx)/d(g22) = ftruey
                // d(Ωmeasx)/d(g23) = ftruez
                // d(Ωmeasx)/d(g31) = 0.0
                // d(Ωmeasx)/d(g32) = 0.0
                // d(Ωmeasx)/d(g33) = 0.0

                // d(Ωmeasz)/d(sx) = 0.0
                // d(Ωmeasz)/d(sy) = 0.0
                // d(Ωmeasz)/d(sz) = Ωtruez
                // d(Ωmeasz)/d(mxy) = 0.0
                // d(Ωmeasz)/d(mxz) = 0.0
                // d(Ωmeasz)/d(myx) = 0.0
                // d(Ωmeasz)/d(myz) = 0.0
                // d(Ωmeasz)/d(mzx) = Ωtruex
                // d(Ωmeasz)/d(mzy) = Ωtruey
                // d(Ωmeasx)/d(g11) = 0.0
                // d(Ωmeasx)/d(g12) = 0.0
                // d(Ωmeasx)/d(g13) = 0.0
                // d(Ωmeasx)/d(g21) = 0.0
                // d(Ωmeasx)/d(g22) = 0.0
                // d(Ωmeasx)/d(g23) = 0.0
                // d(Ωmeasx)/d(g31) = ftruex
                // d(Ωmeasx)/d(g32) = ftruey
                // d(Ωmeasx)/d(g33) = ftruez

                final var sx = params[0];
                final var sy = params[1];
                final var sz = params[2];

                final var mxy = params[3];
                final var mxz = params[4];
                final var myx = params[5];
                final var myz = params[6];
                final var mzx = params[7];
                final var mzy = params[8];

                final var g11 = params[9];
                final var g21 = params[10];
                final var g31 = params[11];
                final var g12 = params[12];
                final var g22 = params[13];
                final var g32 = params[14];
                final var g13 = params[15];
                final var g23 = params[16];
                final var g33 = params[17];

                final var omegatruex = point[0];
                final var omegatruey = point[1];
                final var omegatruez = point[2];

                final var ftruex = point[3];
                final var ftruey = point[4];
                final var ftruez = point[5];

                result[0] = biasX + omegatruex + sx * omegatruex + mxy * omegatruey + mxz * omegatruez
                        + g11 * ftruex + g12 * ftruey + g13 * ftruez;
                result[1] = biasY + myx * omegatruex + omegatruey + sy * omegatruey + myz * omegatruez
                        + g21 * ftruex * g22 * ftruey + g23 * ftruez;
                result[2] = biasZ + mzx * omegatruex + mzy * omegatruey + omegatruez + sz * omegatruez
                        + g31 * ftruex + g32 * ftruey + g33 * ftruez;

                jacobian.setElementAt(0, 0, omegatruex);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, omegatruey);
                jacobian.setElementAt(0, 4, omegatruez);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, 0.0);
                jacobian.setElementAt(0, 7, 0.0);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, ftruex);
                jacobian.setElementAt(0, 10, ftruey);
                jacobian.setElementAt(0, 11, ftruez);
                jacobian.setElementAt(0, 12, 0.0);
                jacobian.setElementAt(0, 13, 0.0);
                jacobian.setElementAt(0, 14, 0.0);
                jacobian.setElementAt(0, 15, 0.0);
                jacobian.setElementAt(0, 16, 0.0);
                jacobian.setElementAt(0, 17, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, omegatruey);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, 0.0);
                jacobian.setElementAt(1, 5, omegatruex);
                jacobian.setElementAt(1, 6, omegatruez);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, 0.0);
                jacobian.setElementAt(1, 9, 0.0);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);
                jacobian.setElementAt(1, 12, ftruex);
                jacobian.setElementAt(1, 13, ftruey);
                jacobian.setElementAt(1, 14, ftruez);
                jacobian.setElementAt(1, 15, 0.0);
                jacobian.setElementAt(1, 16, 0.0);
                jacobian.setElementAt(1, 17, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, omegatruez);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, 0.0);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, omegatruex);
                jacobian.setElementAt(2, 8, omegatruey);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, 0.0);
                jacobian.setElementAt(2, 11, 0.0);
                jacobian.setElementAt(2, 12, 0.0);
                jacobian.setElementAt(2, 13, 0.0);
                jacobian.setElementAt(2, 14, 0.0);
                jacobian.setElementAt(2, 15, ftruex);
                jacobian.setElementAt(2, 16, ftruey);
                jacobian.setElementAt(2, 17, ftruez);
            }
        });

        setInputData();

        fitter.fit();

        final var result = fitter.getA();

        final var sx = result[0];
        final var sy = result[1];
        final var sz = result[2];

        final var mxy = result[3];
        final var mxz = result[4];
        final var myx = result[5];
        final var myz = result[6];
        final var mzx = result[7];
        final var mzy = result[8];

        final var g11 = result[9];
        final var g21 = result[10];
        final var g31 = result[11];
        final var g12 = result[12];
        final var g22 = result[13];
        final var g32 = result[14];
        final var g13 = result[15];
        final var g23 = result[16];
        final var g33 = result[17];

        if (estimatedMg == null) {
            estimatedMg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedMg.initialize(0.0);
        }

        estimatedMg.setElementAt(0, 0, sx);
        estimatedMg.setElementAt(1, 0, myx);
        estimatedMg.setElementAt(2, 0, mzx);

        estimatedMg.setElementAt(0, 1, mxy);
        estimatedMg.setElementAt(1, 1, sy);
        estimatedMg.setElementAt(2, 1, mzy);

        estimatedMg.setElementAt(0, 2, mxz);
        estimatedMg.setElementAt(1, 2, myz);
        estimatedMg.setElementAt(2, 2, sz);

        if (estimatedGg == null) {
            estimatedGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedGg.initialize(0.0);
        }

        estimatedGg.setElementAtIndex(0, g11);
        estimatedGg.setElementAtIndex(1, g21);
        estimatedGg.setElementAtIndex(2, g31);
        estimatedGg.setElementAtIndex(3, g12);
        estimatedGg.setElementAtIndex(4, g22);
        estimatedGg.setElementAtIndex(5, g32);
        estimatedGg.setElementAtIndex(6, g13);
        estimatedGg.setElementAtIndex(7, g23);
        estimatedGg.setElementAtIndex(8, g33);

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
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        final var expectedKinematics = new BodyKinematics();

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, 2 * BodyKinematics.COMPONENTS);
        final var y = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final var standardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredKinematics = measurement.getKinematics();
            final var ecefFrame = measurement.getFrame();
            final var previousEcefFrame = measurement.getPreviousFrame();
            final var timeInterval = measurement.getTimeInterval();

            ECEFKinematicsEstimator.estimateKinematics(timeInterval, ecefFrame, previousEcefFrame, expectedKinematics);

            final var omegaMeasX = measuredKinematics.getAngularRateX();
            final var omegaMeasY = measuredKinematics.getAngularRateY();
            final var omegaMeasZ = measuredKinematics.getAngularRateZ();

            final var omegaTrueX = expectedKinematics.getAngularRateX();
            final var omegaTrueY = expectedKinematics.getAngularRateY();
            final var omegaTrueZ = expectedKinematics.getAngularRateZ();

            final var fTrueX = expectedKinematics.getFx();
            final var fTrueY = expectedKinematics.getFy();
            final var fTrueZ = expectedKinematics.getFz();

            x.setElementAt(i, 0, omegaTrueX);
            x.setElementAt(i, 1, omegaTrueY);
            x.setElementAt(i, 2, omegaTrueZ);

            x.setElementAt(i, 3, fTrueX);
            x.setElementAt(i, 4, fTrueY);
            x.setElementAt(i, 5, fTrueZ);

            y.setElementAt(i, 0, omegaMeasX);
            y.setElementAt(i, 1, omegaMeasY);
            y.setElementAt(i, 2, omegaMeasZ);

            standardDeviations[i] = measurement.getAngularRateStandardDeviation();
            i++;
        }

        fitter.setInputData(x, y, standardDeviations);
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
}
