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
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AccelerationFixer;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.GyroscopeBiasUncertaintySource;
import com.irurueta.navigation.inertial.calibration.GyroscopeCalibrationSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.GradientEstimator;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiDimensionFunctionEvaluator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedConverter;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.Collection;

/**
 * Estimates gyroscope biases, cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator assumes that the IMU is placed flat on a turntable spinning
 * at constant speed, but absolute orientation or position of IMU is unknown.
 * Turntable must rotate fast enough so that Earth rotation effects can be
 * neglected, bus slow enough so that gyroscope readings can be properly made.
 * <p>
 * To use this calibrator at least 10 measurements are needed when common
 * z-axis is assumed and G-dependent cross biases are ignored, otherwise
 * at least 13 measurements are required when common z-axis is not assumed.
 * If G-dependent cross biases are being estimated, then at least 19
 * measurements are needed when common z-axis is assumed, otherwise at
 * least 22 measurements are required when common z-axis is not assumed.
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
public class TurntableGyroscopeCalibrator implements GyroscopeNonLinearCalibrator, UnknownBiasGyroscopeCalibrator,
        GyroscopeCalibrationSource, GyroscopeBiasUncertaintySource,
        UnorderedStandardDeviationBodyKinematicsGyroscopeCalibrator, AccelerometerDependentGyroscopeCalibrator {

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
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope when G-dependent cross biases are being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES = 18;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are being estimated.
     */
    public static final int GENERAL_UNKNOWNS_AND_CROSS_BIASES = 21;

    /**
     * Number of unknowns when common z-axis is assumed for both
     * the accelerometer and gyroscope when G-dependent cross biases
     * are not being estimated.
     */
    public static final int COMMON_Z_AXIS_UNKNOWNS = 9;

    /**
     * Number of unknowns for the general case when G-dependent cross
     * biases are not being estimated.
     */
    public static final int GENERAL_UNKNOWNS = 12;

    /**
     * Required minimum number of measurements when common z-axis is assumed
     * and G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES =
            COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of measurements for the general case and
     * G-dependent cross biases are being estimated.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES = GENERAL_UNKNOWNS_AND_CROSS_BIASES + 1;

    /**
     * Required minimum number of measurements when common z-axis is assumed
     * and G-dependent cross biases are being ignored.
     */
    public static final int MINIMUM_MEASUREMENTS_COMMON_Z_AXIS = COMMON_Z_AXIS_UNKNOWNS + 1;

    /**
     * Required minimum number of measurements for the general case and
     * G-dependent cross biases are being ignored.
     */
    public static final int MINIMUM_MEASUREMENTS_GENERAL = GENERAL_UNKNOWNS + 1;

    /**
     * Default turntable rotation rate.
     */
    public static final double DEFAULT_TURNTABLE_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Default time interval between measurements expressed in seconds (s).
     * This is a typical value when we have 50 samples per second.
     */
    public static final double DEFAULT_TIME_INTERVAL = 0.02;

    /**
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiDimensionFitter fitter = new LevenbergMarquardtMultiDimensionFitter();

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
     * Constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     */
    private double turntableRotationRate = DEFAULT_TURNTABLE_ROTATION_RATE;

    /**
     * Time interval between measurements being captured expressed in
     * second (s).
     */
    private double timeInterval = DEFAULT_TIME_INTERVAL;

    /**
     * Contains a collection of body kinematics measurements taken at
     * a given position with different unknown orientations and containing
     * the standard deviations of accelerometer and gyroscope measurements.
     */
    private Collection<StandardDeviationBodyKinematics> measurements;

    /**
     * Position where body kinematics measures have been taken.
     */
    private ECEFPosition position;

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
     * Listener to handle events raised by this calibrator.
     */
    private TurntableGyroscopeCalibratorListener listener;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
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
     * Internally holds x-coordinate of measured angular rate during calibration.
     */
    private double measAngularRateX;

    /**
     * Internally holds y-coordinate of measured angular rate during calibration.
     */
    private double measAngularRateY;

    /**
     * Internally holds z-coordinate of measured angular rate during calibration.
     */
    private double measAngularRateZ;

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
     * Internally holds measured angular rate during calibration expressed as
     * a column matrix.
     */
    private Matrix measAngularRate;

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
     * Internally hold g-dependent cross biases during calibration.
     */
    private Matrix g;

    /**
     * Internally holds computed true angular rate during calibration.
     */
    private Matrix trueAngularRate;

    /**
     * Internally holds computed true specific force during calibration.
     */
    private Matrix ftrue;

    /**
     * Internally holds accelerometer bias during calibration.
     */
    private Matrix ba;

    /**
     * Internally holds accelerometer scaling and cross coupling errors
     * during calibration.
     */
    private Matrix ma;

    /**
     * Internally holds angular rate bias due to g-dependent cross biases
     */
    private Matrix tmp;

    /**
     * Acceleration fixer.
     */
    private final AccelerationFixer accelerationFixer = new AccelerationFixer();

    /**
     * Constructor.
     */
    public TurntableGyroscopeCalibrator() {
        try {
            initialGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this();
        this.position = position;
        this.measurements = measurements;
        try {
            setTurntableRotationRate(turntableRotationRate);
            setTimeInterval(timeInterval);
            setInitialBias(initialBias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this();
        this.position = position;
        this.measurements = measurements;
        try {
            setTurntableRotationRate(turntableRotationRate);
            setTimeInterval(timeInterval);
            setInitialBias(initialBias);
            setInitialMg(initialMg);
            setInitialGg(initialGg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
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
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must be 3x1
     *                              and is expressed in meters per squared
     *                              second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
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
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg);
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, commonAxisUsed, estimateGDependentCrossBiases,
                initialBias, initialMg, initialGg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final double[] accelerometerBias, final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final double[] accelerometerBias, final Matrix accelerometerMa,
            final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, commonAxisUsed, estimateGDependentCrossBiases,
                initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final Matrix accelerometerBias, final Matrix accelerometerMa) {
        this(position, turntableRotationRate, timeInterval, measurements, initialBias, initialMg, initialGg,
                accelerometerBias, accelerometerMa);
        this.commonAxisUsed = commonAxisUsed;
        this.estimateGDependentCrossBiases = estimateGDependentCrossBiases;
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final ECEFPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final Matrix accelerometerBias, final Matrix accelerometerMa,
            final TurntableGyroscopeCalibratorListener listener) {
        this(position, turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by this
     *                              calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have
     *                              length 3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians
     *                              per second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must have length
     *                              3 and is expressed in radians per
     *                              second (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final double[] initialBias,
            final Matrix initialMg, final Matrix initialGg, final double[] accelerometerBias,
            final Matrix accelerometerMa, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second
     *                              (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position              position where body kinematics measures
     *                              have been taken.
     * @param turntableRotationRate constant rotation rate at which the
     *                              turntable is spinning. Must be
     *                              expressed in radians per second (rad/s).
     * @param timeInterval          time interval between measurements being
     *                              captured expressed in seconds (s).
     * @param measurements          collection of body kinematics
     *                              measurements with standard deviations
     *                              taken at the same position with zero
     *                              velocity and unknown different
     *                              orientations.
     * @param initialBias           initial gyroscope bias to be used to
     *                              find a solution. This must be 3x1 and
     *                              is expressed in radians per second
     *                              (rad/s).
     * @param initialMg             initial gyroscope scale factors and
     *                              cross coupling errors matrix. Must
     *                              be 3x3.
     * @param initialGg             initial gyroscope G-dependent cross
     *                              biases introduced on the gyroscope by
     *                              the specific forces sensed by the
     *                              accelerometer. Must be 3x3.
     * @param accelerometerBias     known accelerometer bias. This must
     *                              have length 3 and is expressed in
     *                              meters per squared second (m/s^2).
     * @param accelerometerMa       known accelerometer scale factors and
     *                              cross coupling matrix. Must be 3x3.
     * @param listener              listener to handle events raised by
     *                              this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final Matrix initialBias,
            final Matrix initialMg, final Matrix initialGg, final Matrix accelerometerBias,
            final Matrix accelerometerMa, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, initialBias, initialMg,
                initialGg, accelerometerBias, accelerometerMa, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific
     *                                      forces sensed by the
     *                                      accelerometer. Must be 3x3.
     * @param listener                      listener to handle events
     *                                      raised by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross
     *                                      biases will be estimated,
     *                                      false otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must have length 3 and is
     *                                      expressed in radians per second
     *                                      (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors
     *                                      matrix. Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer.
     *                                      Must be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final double[] accelerometerBias, final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity
     *                                      and unknown different
     *                                      orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must
     *                                      have length 3 and is expressed
     *                                      in radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final double[] initialBias, final Matrix initialMg,
            final Matrix initialGg, final double[] accelerometerBias, final Matrix accelerometerMa,
            final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be
     *                                      used to find a solution. This
     *                                      must be 3x1 and is expressed in
     *                                      radians per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final Matrix accelerometerBias, final Matrix accelerometerMa) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa);
    }

    /**
     * Constructor.
     *
     * @param position                      position where body kinematics
     *                                      measures have been taken.
     * @param turntableRotationRate         constant rotation rate at which
     *                                      the turntable is spinning. Must
     *                                      be expressed in radians per
     *                                      second (rad/s).
     * @param timeInterval                  time interval between measurements
     *                                      being captured expressed in
     *                                      seconds (s).
     * @param measurements                  collection of body kinematics
     *                                      measurements with standard
     *                                      deviations taken at the same
     *                                      position with zero velocity and
     *                                      unknown different orientations.
     * @param commonAxisUsed                indicates whether z-axis is
     *                                      assumed to be common for
     *                                      accelerometer and gyroscope.
     * @param estimateGDependentCrossBiases true if G-dependent cross biases
     *                                      will be estimated, false
     *                                      otherwise.
     * @param initialBias                   initial gyroscope bias to be used
     *                                      to find a solution. This must be
     *                                      3x1 and is expressed in radians
     *                                      per second (rad/s).
     * @param initialMg                     initial gyroscope scale factors
     *                                      and cross coupling errors matrix.
     *                                      Must be 3x3.
     * @param initialGg                     initial gyroscope G-dependent
     *                                      cross biases introduced on the
     *                                      gyroscope by the specific forces
     *                                      sensed by the accelerometer. Must
     *                                      be 3x3.
     * @param accelerometerBias             known accelerometer bias. This
     *                                      must have length 3 and is
     *                                      expressed in meters per squared
     *                                      second (m/s^2).
     * @param accelerometerMa               known accelerometer scale factors
     *                                      and cross coupling matrix. Must
     *                                      be 3x3.
     * @param listener                      listener to handle events raised
     *                                      by this calibrator.
     * @throws IllegalArgumentException if any of the provided values does
     *                                  not have proper size or if either
     *                                  turntable rotation rate or
     *                                  time interval is zero or negative.
     */
    public TurntableGyroscopeCalibrator(
            final NEDPosition position, final double turntableRotationRate, final double timeInterval,
            final Collection<StandardDeviationBodyKinematics> measurements, final boolean commonAxisUsed,
            final boolean estimateGDependentCrossBiases, final Matrix initialBias, final Matrix initialMg,
            final Matrix initialGg, final Matrix accelerometerBias, final Matrix accelerometerMa,
            final TurntableGyroscopeCalibratorListener listener) {
        this(convertPosition(position), turntableRotationRate, timeInterval, measurements, commonAxisUsed,
                estimateGDependentCrossBiases, initialBias, initialMg, initialGg, accelerometerBias, accelerometerMa,
                listener);
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
            final double accelerometerMxy, final double accelerometerMxz, final double accelerometerMyx,
            final double accelerometerMyz, final double accelerometerMzx, final double accelerometerMzy)
            throws LockedException {
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
            final double accelerometerSx, final double accelerometerSy, final double accelerometerSz,
            final double accelerometerMxy, final double accelerometerMxz, final double accelerometerMyx,
            final double accelerometerMyz, final double accelerometerMzx, final double accelerometerMzy)
            throws LockedException {
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
     * Gets initial x-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    public double getInitialBiasX() {
        return initialBiasX;
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final double initialBiasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = initialBiasX;
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    public double getInitialBiasY() {
        return initialBiasY;
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final double initialBiasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasY = initialBiasY;
    }

    /**
     * Gets initial z-coordinate of gyroscope bias ot be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    public double getInitialBiasZ() {
        return initialBiasZ;
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find
     * a solution.
     * This is expressed in radians per second (rad/s).
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasZ(final double initialBiasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasZ = initialBiasZ;
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial x-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedX() {
        return new AngularSpeed(initialBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedX(final AngularSpeed result) {
        result.setValue(initialBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial x-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasX(final AngularSpeed initialBiasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasX = convertAngularSpeed(initialBiasX);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial y-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedY() {
        return new AngularSpeed(initialBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedY(final AngularSpeed result) {
        result.setValue(initialBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial y-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
    public void setInitialBiasY(final AngularSpeed initialBiasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.initialBiasY = convertAngularSpeed(initialBiasY);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @return initial z-coordinate of gyroscope bias.
     */
    public AngularSpeed getInitialBiasAngularSpeedZ() {
        return new AngularSpeed(initialBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param result instance where result data will be stored.
     */
    public void getInitialBiasAngularSpeedZ(final AngularSpeed result) {
        result.setValue(initialBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets initial z-coordinate of gyroscope bias to be used to find a
     * solution.
     *
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
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
     * Sets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param initialBiasX initial x-coordinate of gyroscope bias.
     * @param initialBiasY initial y-coordinate of gyroscope bias.
     * @param initialBiasZ initial z-coordinate of gyroscope bias.
     * @throws LockedException if calibrator is currently running.
     */
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
     * Gets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @return array containing coordinates of initial gyroscope bias.
     */
    public double[] getInitialBias() {
        final var result = new double[BodyKinematics.COMPONENTS];
        getInitialBias(result);
        return result;
    }

    /**
     * Gets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getInitialBias(final double[] result) {
        if (result.length != BodyKinematics.COMPONENTS) {
            throw new IllegalArgumentException();
        }
        result[0] = initialBiasX;
        result[1] = initialBiasY;
        result[2] = initialBiasZ;
    }

    /**
     * Sets initial gyroscope bias to be used to find a solution as
     * an array.
     * Array values are expressed in radians per second (rad/s).
     *
     * @param initialBias initial bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
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
     * Gets initial gyroscope bias to be used to find a solution as a
     * column matrix.
     *
     * @return initial gyroscope bias to be used to find a solution as a
     * column matrix.
     */
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
     * Gets initial gyroscope bias to be used to find a solution as a
     * column matrix.
     *
     * @param result instance where result data will be copied to.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public void getInitialBiasAsMatrix(final Matrix result) {
        if (result.getRows() != BodyKinematics.COMPONENTS || result.getColumns() != 1) {
            throw new IllegalArgumentException();
        }
        result.setElementAtIndex(0, initialBiasX);
        result.setElementAtIndex(1, initialBiasY);
        result.setElementAtIndex(2, initialBiasZ);
    }

    /**
     * Sets initial gyroscope bias to be used to find a solution as
     * an array.
     *
     * @param initialBias initial gyroscope bias to find a solution.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
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
    public AngularSpeedTriad getInitialBiasAsTriad() {
        return new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, initialBiasX, initialBiasY, initialBiasZ);
    }

    /**
     * Gets initial bias coordinates of gyroscope used to find a solution.
     *
     * @param result instance where result will be stored.
     */
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
    public void setInitialBias(final AngularSpeedTriad initialBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        initialBiasX = convertAngularSpeed(initialBias.getValueX(), initialBias.getUnit());
        initialBiasY = convertAngularSpeed(initialBias.getValueY(), initialBias.getUnit());
        initialBiasZ = convertAngularSpeed(initialBias.getValueZ(), initialBias.getUnit());
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
     * Gets constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     *
     * @return constant rotation rate of turntable.
     */
    public double getTurntableRotationRate() {
        return turntableRotationRate;
    }

    /**
     * Sets constant rotation rate at which the turntable is spinning.
     * This is expressed in radians per second (rad/s).
     *
     * @param turntableRotationRate constant rotation rate of turntable.
     * @throws LockedException          if calibrator is currently running
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTurntableRotationRate(final double turntableRotationRate) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (turntableRotationRate <= 0.0) {
            throw new IllegalArgumentException();
        }

        this.turntableRotationRate = turntableRotationRate;
    }

    /**
     * Gets constant rotation rate at which the turntable is spinning.
     *
     * @return constant rotation rate of turntable.
     */
    public AngularSpeed getTurntableRotationRateAsAngularSpeed() {
        return new AngularSpeed(turntableRotationRate, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets constant rotation rate at which the turntable is spinning.
     *
     * @param result instance where result will be stored.
     */
    public void getTurntableRotationRateAsAngularSpeed(final AngularSpeed result) {
        result.setValue(turntableRotationRate);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets constant rotation rate at which the turntable is spinning.
     *
     * @param turntableRotationRate constant rotation rate of turntable.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTurntableRotationRate(final AngularSpeed turntableRotationRate) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        setTurntableRotationRate(convertAngularSpeed(turntableRotationRate));
    }

    /**
     * Gets time interval between measurements being captured expressed in
     * seconds (s).
     *
     * @return time interval between measurements.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets time interval between measurements being captured expressed in
     * seconds (s).
     *
     * @param timeInterval time interval between measurements.
     * @throws LockedException          if calibrator is currently running.
     * @throws IllegalArgumentException if provided value is zero or
     *                                  negative.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (timeInterval <= 0.0) {
            throw new IllegalArgumentException();
        }
        this.timeInterval = timeInterval;
    }

    /**
     * Gets time interval between measurements being captured.
     *
     * @return time interval between measurements.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between measurements being captured.
     *
     * @param result instance where result will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between measurements being captured.
     *
     * @param timeInterval time interval between measurements.
     * @throws LockedException if calibrator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        setTimeInterval(convertTime(timeInterval));
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
     * Gets position where body kinematics measures have been taken expressed in
     * ECEF coordinates.
     *
     * @return position where body kinematics measures have been taken.
     */
    public ECEFPosition getEcefPosition() {
        return position;
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * ECEF coordinates.
     *
     * @param position position where body kinematics measures have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final ECEFPosition position) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.position = position;
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @return position where body kinematics measures have been taken or null if
     * not available.
     */
    public NEDPosition getNedPosition() {
        final var result = new NEDPosition();
        return getNedPosition(result) ? result : null;
    }

    /**
     * Gets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @param result instance where result will be stored.
     * @return true if NED position could be computed, false otherwise.
     */
    public boolean getNedPosition(final NEDPosition result) {
        if (position != null) {
            final var velocity = new NEDVelocity();
            ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                    position.getX(), position.getY(), position.getZ(),
                    0.0, 0.0, 0.0, result, velocity);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets position where body kinematics measures have been taken expressed in
     * NED coordinates.
     *
     * @param position position where body kinematics measures have been taken.
     * @throws LockedException if calibrator is currently running.
     */
    public void setPosition(final NEDPosition position) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.position = convertPosition(position);
    }

    /**
     * Indicates the type of measurement or sequence used by this calibrator.
     *
     * @return type of measurement or sequence used by this calibrator.
     */
    @Override
    public GyroscopeCalibratorMeasurementOrSequenceType getMeasurementOrSequenceType() {
        return GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_BODY_KINEMATICS_MEASUREMENT;
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
    public TurntableGyroscopeCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if calibrator is currently running.
     */
    public void setListener(final TurntableGyroscopeCalibratorListener listener) throws LockedException {
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
        if (commonAxisUsed) {
            if (estimateGDependentCrossBiases) {
                return MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES;
            } else {
                return MINIMUM_MEASUREMENTS_COMMON_Z_AXIS;
            }
        } else {
            if (estimateGDependentCrossBiases) {
                return MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES;
            } else {
                return MINIMUM_MEASUREMENTS_GENERAL;
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
        return measurements != null && measurements.size() >= getMinimumRequiredMeasurementsOrSequences();
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
     * cross-coupling errors and G-dependent coupling.
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
                if (estimateGDependentCrossBiases) {
                    calibrateCommonAxisAndGDependentCrossBiases();
                } else {
                    calibrateCommonAxis();
                }
            } else {
                if (estimateGDependentCrossBiases) {
                    calibrateGeneralAndGDependentCrossBiases();
                } else {
                    calibrateGeneral();
                }
            }

            if (listener != null) {
                listener.onCalibrateEnd(this);
            }

        } catch (final AlgebraException | FittingException | com.irurueta.numerical.NotReadyException |
                       InvalidSourceAndDestinationFrameTypeException e) {
            throw new CalibrationException(e);
        } finally {
            running = false;
        }
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
            result.setValueCoordinatesAndUnit(estimatedBiases[0], estimatedBiases[1], estimatedBiases[2],
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
     * parameters (following indicated order): bgx, bgy, bgz, sx, sy, sz,
     * mxy, mxz, myx, myz, mzx, mzy, gg11, gg21, gg31, gg12, gg22, gg32,
     * gg13, gg23, gg33.
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
                ? new AngularSpeed(getEstimatedBiasXStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND)
                : null;
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
                ? new AngularSpeed(getEstimatedBiasYStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND)
                : null;
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
                ? new AngularSpeed(getEstimatedBiasZStandardDeviation(), AngularSpeedUnit.RADIANS_PER_SECOND)
                : null;
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
                + getEstimatedBiasZStandardDeviation()) / 3.0
                : null;
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
     * Internal method to perform calibration when common z-axis is assumed
     * for both the accelerometer and gyroscope and when G-dependent cross
     * biases are being estimated.
     *
     * @throws AlgebraException                              if there are numerical errors.
     * @throws FittingException                              if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens
     */
    private void calibrateCommonAxisAndGDependentCrossBiases() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, InvalidSourceAndDestinationFrameTypeException {

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, we
        // take common factor M = I + Mg

        // and the gyroscope model can be better expressed as:

        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // where:
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // We know that the norm of the true angular rate when the device is in a fixed
        // and unknown position and orientation is equal to the Earth rotation rate.
        // ||Ωtrue|| = 7.292115E-5 rad/s

        // Hence
        // Ωmeas - M*b - M*G*ftrue = M*Ωtrue
        // M^-1 * (Ωmeas - M*b - M*G*ftrue) = Ωtrue

        // ||Ωtrue||^2 = (M^-1 * (Ωmeas - M*b - M*G*ftrue))^T*(M^-1 * (Ωmeas - M*b - M*G*ftrue))
        // ||Ωtrue||^2 = (Ωmeas - M*b - M*G*ftrue)^T * (M^-1)^T * M^-1 * (Ωmeas - M*b - M*G*ftrue)
        // ||Ωtrue||^2 = (Ωmeas - M*b - M*G*ftrue)^T * ||M^-1||^2 * (Ωmeas - M*b - M*G*ftrue)
        // ||Ωtrue||^2 = ||Ωmeas - M*b - M*G*ftrue||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]

        // G = [g11 	g12 	g13]
        //     [g21 	g22 	g23]
        //     [g31 	g32 	g33]

        // ftrue = [ftruex]
        //         [ftruey]
        //         [fturez]

        final var gradientEstimator = new GradientEstimator(this::evaluateCommonAxisWithGDependentCrossBiases);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final var invInitM = Utils.inverse(initialM);
        final var initBg = getInitialBiasAsMatrix();
        final var initB = invInitM.multiplyAndReturnNew(initBg);
        final var initGg = getInitialGg();
        final var initG = invInitM.multiplyAndReturnNew(initGg);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured angular rate coordinates +
                // measured specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES];

                // biases b
                for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
                    initial[i] = initB.getElementAtIndex(i);
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

                // g-dependent cross biases G
                final var num = BodyKinematics.COMPONENTS * BodyKinematics.COMPONENTS;
                for (int i = 0, j = k; i < num; i++, j++) {
                    initial[j] = initG.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives)
                    throws EvaluationException {

                measAngularRateX = point[0];
                measAngularRateY = point[1];
                measAngularRateZ = point[2];

                fmeasX = point[3];
                fmeasY = point[4];
                fmeasZ = point[5];

                gradientEstimator.gradient(params, derivatives);

                return evaluateCommonAxisWithGDependentCrossBiases(params);
            }
        });

        setInputDataWithGDependentCrossBiases();

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

        final var g11 = result[9];
        final var g21 = result[10];
        final var g31 = result[11];

        final var g12 = result[12];
        final var g22 = result[13];
        final var g32 = result[14];

        final var g13 = result[15];
        final var g23 = result[16];
        final var g33 = result[17];

        final var mb = new Matrix(BodyKinematics.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        mm.setElementAtIndex(0, m11);
        mm.setElementAtIndex(1, 0.0);
        mm.setElementAtIndex(2, 0.0);

        mm.setElementAtIndex(3, m12);
        mm.setElementAtIndex(4, m22);
        mm.setElementAtIndex(5, 0.0);

        mm.setElementAtIndex(6, m13);
        mm.setElementAtIndex(7, m23);
        mm.setElementAtIndex(8, m33);

        final var mg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        mg.setElementAtIndex(0, g11);
        mg.setElementAtIndex(1, g21);
        mg.setElementAtIndex(2, g31);

        mg.setElementAtIndex(3, g12);
        mg.setElementAtIndex(4, g22);
        mg.setElementAtIndex(5, g32);

        mg.setElementAtIndex(6, g13);
        mg.setElementAtIndex(7, g23);
        mg.setElementAtIndex(8, g33);

        setResult(mm, mb, mg);

        // at this point covariance is expressed in terms of b, M and G, and must
        // be expressed in terms of bg, Mg and Gg.
        // We know that:
        // bg = M * b
        // Mg = M - I
        // Gg = M * G

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11  m12  m13]
        //     [0    m22  m23]
        //     [0    0    m33]

        // G = [g11  g12  g13]
        //     [g21  g22  g23]
        //     [g31  g32  g33]

        // bg = [m11  m12  m13][bx] = [m11 * bx + m12 * by + m13 * bz] = [bgx]
        //      [0    m22  m23][by]   [           m22 * by + m23 * bz]   [bgy]
        //      [0    0    m33][bz]   [                      m33 * bz]   [bgz]

        // Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //      [myx sy	  myz]   [0          m22 - 1     m23     ]
        //      [mzx mzy  sz ]   [0          0           m33 - 1 ]

        // Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //      [gg21  gg22  gg23]   [0    m22  m23][g21  g22  g23]
        //      [gg31  gg32  gg33]   [0    0    m33][g31  g32  g33]

        // Defining the linear application:
        // F(b, M, G) = F(bx, by, bz, m11, m12, m22, m13, m23, m33, g11, g21, g31, g12, g22, g32, g13, g23, g33)
        // as:
        // [bgx] =  [m11 * bx + m12 * by + m13 * bz]
        // [bgy]    [           m22 * by + m23 * bz]
        // [bgz]    [                      m33 * bz]
        // [sx]     [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [0]
        // [myz]    [m23]
        // [mzx]    [0]
        // [mzy]    [0]
        // [gg11]   [m11 * g11 + m12 * g21 + m13 * g31]
        // [gg21]   [            m22 * g21 + m23 * g31]
        // [gg31]   [                        m33 * g31]
        // [gg12]   [m11 * g12 + m12 * g22 + m13 * g32]
        // [gg22]   [            m22 * g22 + m23 * g32]
        // [gg32]   [                        m33 * g32]
        // [gg13]   [m11 * g13 + m12 * g23 + m13 * g33]
        // [gg23]   [            m22 * g23 + m23 * g33]
        // [gg33]   [                        m33 * g33]

        // Then the Jacobian of F(b, M, G) is:
        // J = [m11  m12  m13  bx   by   0    bz   0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    m22  m23  0    0    by   0    bz   0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    m33  0    0    0    0    0    bz   0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    g11  g21  0    g31  0    0    m11  m12  m13  0    0    0    0    0    0  ]
        //     [0    0    0    0    0    g21  0    g31  0    0    m22  m23  0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    g31  0    0    m33  0    0    0    0    0    0  ]
        //     [0    0    0    g12  g22  0    g32  0    0    0    0    0    m11  m12  m13  0    0    0  ]
        //     [0    0    0    0    0    g22  0    g32  0    0    0    0    0    m22  m23  0    0    0  ]
        //     [0    0    0    0    0    0    0    0    g32  0    0    0    0    0    m33  0    0    0  ]
        //     [0    0    0    g13  g23  0    g33  0    0    0    0    0    0    0    0    m11  m12  m13]
        //     [0    0    0    0    0    g23  0    g33  0    0    0    0    0    0    0    0    m22  m23]
        //     [0    0    0    0    0    0    0    0    g33  0    0    0    0    0    0    0    0    m33]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, COMMON_Z_AXIS_UNKNOWNS_AND_CROSS_BIASES);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(0, 4, by);
        jacobian.setElementAt(0, 6, bz);

        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(1, 5, by);
        jacobian.setElementAt(1, 7, bz);

        jacobian.setElementAt(2, 2, m33);
        jacobian.setElementAt(2, 8, bz);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 5, 1.0);
        jacobian.setElementAt(5, 8, 1.0);

        jacobian.setElementAt(6, 4, 1.0);
        jacobian.setElementAt(7, 6, 1.0);

        jacobian.setElementAt(9, 7, 1.0);

        jacobian.setElementAt(12, 3, g11);
        jacobian.setElementAt(12, 4, g21);
        jacobian.setElementAt(12, 6, g31);
        jacobian.setElementAt(12, 9, m11);
        jacobian.setElementAt(12, 10, m12);
        jacobian.setElementAt(12, 11, m13);

        jacobian.setElementAt(13, 5, g21);
        jacobian.setElementAt(13, 7, g31);
        jacobian.setElementAt(13, 10, m22);
        jacobian.setElementAt(13, 11, m23);

        jacobian.setElementAt(14, 8, g31);
        jacobian.setElementAt(14, 11, m33);

        jacobian.setElementAt(15, 3, g12);
        jacobian.setElementAt(15, 4, g22);
        jacobian.setElementAt(15, 6, g32);
        jacobian.setElementAt(15, 12, m11);
        jacobian.setElementAt(15, 13, m12);
        jacobian.setElementAt(15, 14, m13);

        jacobian.setElementAt(16, 5, g22);
        jacobian.setElementAt(16, 7, g32);
        jacobian.setElementAt(16, 13, m22);
        jacobian.setElementAt(16, 14, m23);

        jacobian.setElementAt(17, 8, g32);
        jacobian.setElementAt(17, 14, m33);

        jacobian.setElementAt(18, 3, g13);
        jacobian.setElementAt(18, 4, g23);
        jacobian.setElementAt(18, 6, g33);
        jacobian.setElementAt(18, 15, m11);
        jacobian.setElementAt(18, 16, m12);
        jacobian.setElementAt(18, 17, m13);

        jacobian.setElementAt(19, 5, g23);
        jacobian.setElementAt(19, 7, g33);
        jacobian.setElementAt(19, 16, m22);
        jacobian.setElementAt(19, 17, m23);

        jacobian.setElementAt(20, 8, g33);
        jacobian.setElementAt(20, 17, m33);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform general calibration when G-dependent cross
     * biases are being estimated.
     *
     * @throws AlgebraException                              if there are numerical errors.
     * @throws FittingException                              if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens
     */
    private void calibrateGeneralAndGDependentCrossBiases() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, InvalidSourceAndDestinationFrameTypeException {

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, we
        // take common factor M = I + Mg

        // and the gyroscope model can be better expressed as:

        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // where:
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // We know that the norm of the true angular rate when the device is in a pixed
        // and unknown position and orientation is equal to the Earth rotation rate.
        // ||Ωtrue|| = 7.292115E-5 rad/s

        // Hence
        // Ωmeas - M*b - M*G*ftrue = M*Ωtrue
        // M^-1 * (Ωmeas - M*b - M*G*ftrue) = Ωtrue

        // ||Ωtrue||^2 = (M^-1 * (Ωmeas - M*b - M*G*ftrue))^T*(M^-1 * (Ωmeas - M*b - M*G*ftrue))
        // ||Ωtrue||^2 = (Ωmeas - M*b - M*G*ftrue)^T * (M^-1)^T * M^-1 * (Ωmeas - M*b - M*G*ftrue)
        // ||Ωtrue||^2 = (Ωmeas - M*b - M*G*ftrue)^T * ||M^-1||^2 * (Ωmeas - M*b - M*G*ftrue)
        // ||Ωtrue||^2 = ||Ωmeas - M*b - M*G*ftrue||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        // G = [g11 	g12 	g13]
        //     [g21 	g22 	g23]
        //     [g31 	g32 	g33]

        // ftrue = [ftruex]
        //         [ftruey]
        //         [fturez]

        final var gradientEstimator = new GradientEstimator(this::evaluateGeneralWithGDependentCrossBiases);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final var invInitM = Utils.inverse(initialM);
        final var initBg = getInitialBiasAsMatrix();
        final var initB = invInitM.multiplyAndReturnNew(initBg);
        final var initGg = getInitialGg();
        final var initG = invInitM.multiplyAndReturnNew(initGg);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured angular rate coordinates +
                // measured specific force coordinates
                return 2 * BodyKinematics.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS_AND_CROSS_BIASES];

                // biases b
                for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
                    initial[i] = initB.getElementAtIndex(i);
                }

                // cross coupling errors M
                final var num = BodyKinematics.COMPONENTS * BodyKinematics.COMPONENTS;
                for (int i = 0, j = BodyKinematics.COMPONENTS; i < num; i++, j++) {
                    initial[j] = initialM.getElementAtIndex(i);
                }

                // g-dependent cross biases G
                for (int i = 0, j = BodyKinematics.COMPONENTS + num; i < num; i++, j++) {
                    initial[j] = initG.getElementAtIndex(i);
                }

                return initial;
            }

            @Override
            public double evaluate(
                    final int i, final double[] point, final double[] params, final double[] derivatives)
                    throws EvaluationException {

                measAngularRateX = point[0];
                measAngularRateY = point[1];
                measAngularRateZ = point[2];

                fmeasX = point[3];
                fmeasY = point[4];
                fmeasZ = point[5];

                gradientEstimator.gradient(params, derivatives);

                return evaluateGeneralWithGDependentCrossBiases(params);
            }
        });

        setInputDataWithGDependentCrossBiases();

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

        final var g11 = result[12];
        final var g21 = result[13];
        final var g31 = result[14];

        final var g12 = result[15];
        final var g22 = result[16];
        final var g32 = result[17];

        final var g13 = result[18];
        final var g23 = result[19];
        final var g33 = result[20];

        final var mb = new Matrix(BodyKinematics.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        mm.setElementAtIndex(0, m11);
        mm.setElementAtIndex(1, m21);
        mm.setElementAtIndex(2, m31);

        mm.setElementAtIndex(3, m12);
        mm.setElementAtIndex(4, m22);
        mm.setElementAtIndex(5, m32);

        mm.setElementAtIndex(6, m13);
        mm.setElementAtIndex(7, m23);
        mm.setElementAtIndex(8, m33);

        final var mg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        mg.setElementAtIndex(0, g11);
        mg.setElementAtIndex(1, g21);
        mg.setElementAtIndex(2, g31);

        mg.setElementAtIndex(3, g12);
        mg.setElementAtIndex(4, g22);
        mg.setElementAtIndex(5, g32);

        mg.setElementAtIndex(6, g13);
        mg.setElementAtIndex(7, g23);
        mg.setElementAtIndex(8, g33);

        setResult(mm, mb, mg);

        // at this point covariance is expressed in terms of b, M and G, and must
        // be expressed in terms of bg, Mg and Gg.
        // We know that:
        // bg = M * b
        // Mg = M - I
        // Gg = M * G

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11  m12  m13]
        //     [m21  m22  m23]
        //     [m31  m32  m33]

        // G = [g11  g12  g13]
        //     [g21  g22  g23]
        //     [g31  g32  g33]

        // bg = [m11  m12  m13][bx] = [m11 * bx + m12 * by + m13 * bz] = [bgx]
        //      [m21  m22  m23][by]   [m21 * bx + m22 * by + m23 * bz]   [bgy]
        //      [m31  m32  m33][bz]   [m31 * bx + m32 * by + m33 * bz]   [bgz]

        // Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //      [myx sy	  myz]   [m21        m22 - 1     m23     ]
        //      [mzx mzy  sz ]   [m31        m32         m33 - 1 ]

        // Gg = [gg11  gg12  gg13] = [m11  m12  m13][g11  g12  g13]
        //      [gg21  gg22  gg23]   [m21  m22  m23][g21  g22  g23]
        //      [gg31  gg32  gg33]   [m31  m32  m33][g31  g32  g33]

        // Defining the linear application:
        // F(b, M, G) = F(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33, g11, g21, g31, g12, g22, g32, g13, g23, g33)
        // as:
        // [bgx] =  [m11 * bx + m12 * by + m13 * bz]
        // [bgy]    [m21 * bx + m22 * by + m23 * bz]
        // [bgz]    [m31 * bx + m32 * by + m33 * bz]
        // [sx]     [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [m21]
        // [myz]    [m23]
        // [mzx]    [m31]
        // [mzy]    [m32]
        // [gg11]   [m11 * g11 + m12 * g21 + m13 * g31]
        // [gg21]   [m21 * g11 + m22 * g21 + m23 * g31]
        // [gg31]   [m31 * g11 + m32 * g21 + m33 * g31]
        // [gg12]   [m11 * g12 + m12 * g22 + m13 * g32]
        // [gg22]   [m21 * g12 + m22 * g22 + m23 * g32]
        // [gg32]   [m31 * g12 + m32 * g22 + m33 * g32]
        // [gg13]   [m11 * g13 + m12 * g23 + m13 * g33]
        // [gg23]   [m21 * g13 + m22 * g23 + m23 * g33]
        // [gg33]   [m31 * g13 + m32 * g23 + m33 * g33]

        // Then the Jacobian of F(b, M, G) is:
        // J = [m11  m12  m13  bx   0    0    by   0    0    bz   0    0    0    0    0    0    0    0    0    0    0  ]
        //     [m21  m22  m23  0    bx   0    0    by   0    0    bz   0    0    0    0    0    0    0    0    0    0  ]
        //     [m31  m32  m33  0    0    bx   0    0    by   0    0    bz   0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    1    0    0    0    0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    g11  0    0    g21  0    0    g31  0    0    m11  m12  m13  0    0    0    0    0    0  ]
        //     [0    0    0    0    g11  0    0    g21  0    0    g31  0    m21  m22  m23  0    0    0    0    0    0  ]
        //     [0    0    0    0    0    g11  0    0    g21  0    0    g31  m31  m32  m33  0    0    0    0    0    0  ]
        //     [0    0    0    g12  0    0    g22  0    0    g32  0    0    0    0    0    m11  m12  m13  0    0    0  ]
        //     [0    0    0    0    g12  0    0    g22  0    0    g32  0    0    0    0    m21  m22  m23  0    0    0  ]
        //     [0    0    0    0    0    g12  0    0    g22  0    0    g32  0    0    0    m31  m32  m33  0    0    0  ]
        //     [0    0    0    g13  0    0    g23  0    0    g33  0    0    0    0    0    0    0    0    m11  m12  m13]
        //     [0    0    0    0    g13  0    0    g23  0    0    g33  0    0    0    0    0    0    0    m21  m22  m23]
        //     [0    0    0    0    0    g13  0    0    g23  0    0    g33  0    0    0    0    0    0    m31  m32  m33]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, GENERAL_UNKNOWNS_AND_CROSS_BIASES);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(0, 6, by);
        jacobian.setElementAt(0, 9, bz);

        jacobian.setElementAt(1, 0, m21);
        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(1, 4, bx);
        jacobian.setElementAt(1, 7, by);
        jacobian.setElementAt(1, 10, bz);

        jacobian.setElementAt(2, 0, m31);
        jacobian.setElementAt(2, 1, m32);
        jacobian.setElementAt(2, 2, m33);
        jacobian.setElementAt(2, 5, bx);
        jacobian.setElementAt(2, 8, by);
        jacobian.setElementAt(2, 11, bz);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 7, 1.0);
        jacobian.setElementAt(5, 11, 1.0);

        jacobian.setElementAt(6, 6, 1.0);
        jacobian.setElementAt(7, 9, 1.0);
        jacobian.setElementAt(8, 4, 1.0);

        jacobian.setElementAt(9, 10, 1.0);
        jacobian.setElementAt(10, 5, 1.0);
        jacobian.setElementAt(11, 8, 1.9);

        jacobian.setElementAt(12, 3, g11);
        jacobian.setElementAt(12, 6, g21);
        jacobian.setElementAt(12, 9, g31);
        jacobian.setElementAt(12, 12, m11);
        jacobian.setElementAt(12, 13, m12);
        jacobian.setElementAt(12, 14, m13);

        jacobian.setElementAt(13, 4, g11);
        jacobian.setElementAt(13, 7, g21);
        jacobian.setElementAt(13, 10, g31);
        jacobian.setElementAt(13, 12, m21);
        jacobian.setElementAt(13, 13, m22);
        jacobian.setElementAt(13, 14, m23);

        jacobian.setElementAt(14, 5, g11);
        jacobian.setElementAt(14, 8, g21);
        jacobian.setElementAt(14, 11, g31);
        jacobian.setElementAt(14, 12, m31);
        jacobian.setElementAt(14, 13, m32);
        jacobian.setElementAt(14, 14, m33);

        jacobian.setElementAt(15, 3, g12);
        jacobian.setElementAt(15, 6, g22);
        jacobian.setElementAt(15, 9, g32);
        jacobian.setElementAt(15, 15, m11);
        jacobian.setElementAt(15, 16, m12);
        jacobian.setElementAt(15, 17, m13);

        jacobian.setElementAt(16, 4, g12);
        jacobian.setElementAt(16, 7, g22);
        jacobian.setElementAt(16, 10, g32);
        jacobian.setElementAt(16, 15, m21);
        jacobian.setElementAt(16, 16, m22);
        jacobian.setElementAt(16, 17, m23);

        jacobian.setElementAt(17, 5, g12);
        jacobian.setElementAt(17, 8, g22);
        jacobian.setElementAt(17, 11, g32);
        jacobian.setElementAt(17, 15, m31);
        jacobian.setElementAt(17, 16, m32);
        jacobian.setElementAt(17, 17, m33);

        jacobian.setElementAt(18, 3, g13);
        jacobian.setElementAt(18, 6, g23);
        jacobian.setElementAt(18, 9, g33);
        jacobian.setElementAt(18, 18, m11);
        jacobian.setElementAt(18, 19, m12);
        jacobian.setElementAt(18, 20, m13);

        jacobian.setElementAt(19, 4, g13);
        jacobian.setElementAt(19, 7, g23);
        jacobian.setElementAt(19, 10, g33);
        jacobian.setElementAt(19, 18, m21);
        jacobian.setElementAt(19, 19, m22);
        jacobian.setElementAt(19, 20, m23);

        jacobian.setElementAt(20, 5, g13);
        jacobian.setElementAt(20, 8, g23);
        jacobian.setElementAt(20, 11, g33);
        jacobian.setElementAt(20, 18, m31);
        jacobian.setElementAt(20, 19, m32);
        jacobian.setElementAt(20, 20, m33);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope and G-dependent cross biases are ignored.
     *
     * @throws AlgebraException                              if there are numerical errors.
     * @throws FittingException                              if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, InvalidSourceAndDestinationFrameTypeException {

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Since G-dependent cross biases are ignored, we can assume that Gg = 0

        // Hence:
        // Ωmeas = bg + (I + Mg) * Ωtrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // gyroscope model can be better expressed as:
        // Ωmeas = T*K*(Ωtrue + b)
        // Ωmeas = M*(Ωtrue + b)
        // Ωmeas = M*Ωtrue + M*b

        // where:
        // M = I + Mg
        // bg = M*b = (I + Mg)*b --> b = M^-1*bg

        // We know that the norm of the true angular rate when the device is in a pixed
        // and unknown position and orientation is equal to the Earth rotation rate.
        // ||Ωtrue|| = 7.292115E-5 rad/s

        // Hence
        // Ωmeas - M*b = M*Ωtrue

        // M^-1 * (Ωmeas - M*b) = Ωtrue

        // ||Ωtrue||^2 = (M^-1 * (Ωmeas - M*b))^T*(M^-1 * (Ωmeas - M*b))
        // ||Ωtrue||^2 = (Ωmeas - M*b)^T * (M^-1)^T * M^-1 * (Ωmeas - M*b)
        // ||Ωtrue||^2 = (Ωmeas - M*b)^T * ||M^-1||^2 * (Ωmeas - M*b)
        // ||Ωtrue||^2 = ||Ωmeas - M*b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [0 		m22 	m23]
        //     [0 	 	0 		m33]

        final var gradientEstimator = new GradientEstimator(this::evaluateCommonAxis);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        // Force initial M to be upper diagonal
        initialM.setElementAt(1, 0, 0.0);
        initialM.setElementAt(2, 0, 0.0);
        initialM.setElementAt(2, 1, 0.0);

        final var invInitialM = Utils.inverse(initialM);
        final var initialBg = getInitialBiasAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBg);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured angular rate coordinates
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

                measAngularRateX = point[0];
                measAngularRateY = point[1];
                measAngularRateZ = point[2];

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

        final var mb = new Matrix(BodyKinematics.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
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

        // at this point covariance is expressed in terms of b, M and G, and must
        // be expressed in terms of bg, Mg and Gg.
        // We know that:
        // bg = M * b
        // Mg = M - I
        // Gg = M * G = 0

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11  m12  m13]
        //     [0    m22  m23]
        //     [0    0    m33]

        // G = [g11  g12  g13] = 0
        //     [g21  g22  g23]
        //     [g31  g32  g33]

        // bg = [m11  m12  m13][bx] = [m11 * bx + m12 * by + m13 * bz] = [bgx]
        //      [0    m22  m23][by]   [           m22 * by + m23 * bz]   [bgy]
        //      [0    0    m33][bz]   [                      m33 * bz]   [bgz]

        // Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //      [myx sy	  myz]   [0          m22 - 1     m23     ]
        //      [mzx mzy  sz ]   [0          0           m33 - 1 ]

        //Gg = [gg11  gg12  gg13] = 0
        //     [gg21  gg22  gg23]
        //     [gg31  gg32  gg33]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m12, m22, m13, m23, m33)
        // as:
        // [bgx] =  [m11 * bx + m12 * by + m13 * bz]
        // [bgy]    [           m22 * by + m23 * bz]
        // [bgz]    [                      m33 * bz]
        // [sx]     [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [0]
        // [myz]    [m23]
        // [mzx]    [0]
        // [mzy]    [0]
        // [gg11]   [0]
        // [gg21]   [0]
        // [gg31]   [0]
        // [gg12]   [0]
        // [gg22]   [0]
        // [gg32]   [0]
        // [gg13]   [0]
        // [gg23]   [0]
        // [gg33]   [0]

        // Then the Jacobian of F(b, M) is:
        // J = [m11  m12  m13  bx   by   0    bz   0    0  ]
        //     [0    m22  m23  0    0    by   0    bz   0  ]
        //     [0    0    m33  0    0    0    0    0    bz ]
        //     [0    0    0    1    0    0    0    0    0  ]
        //     [0    0    0    0    0    1    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    1  ]
        //     [0    0    0    0    1    0    0    0    0  ]
        //     [0    0    0    0    0    0    1    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    1    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]
        //     [0    0    0    0    0    0    0    0    0  ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, COMMON_Z_AXIS_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(0, 4, by);
        jacobian.setElementAt(0, 6, bz);

        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(1, 5, by);
        jacobian.setElementAt(1, 7, bz);

        jacobian.setElementAt(2, 2, m33);
        jacobian.setElementAt(2, 8, bz);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 5, 1.0);
        jacobian.setElementAt(5, 8, 1.0);

        jacobian.setElementAt(6, 4, 1.0);
        jacobian.setElementAt(7, 6, 1.0);

        jacobian.setElementAt(9, 7, 1.0);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Internal method to perform general calibration when G-dependent cross biases
     * are ignored.
     *
     * @throws AlgebraException                              if there are numerical errors.
     * @throws FittingException                              if no convergence to solution is found.
     * @throws com.irurueta.numerical.NotReadyException      if fitter is not ready.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException, com.irurueta.numerical.NotReadyException,
            InvalidSourceAndDestinationFrameTypeException {

        // The gyroscope model is
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Since G-dependent cross biases are ignored, we can assume that Gg = 0

        // Hence:
        // Ωmeas = bg + (I + Mg) * Ωtrue

        // For convergence purposes of the Levenberg-Marquardt algorithm, the
        // gyroscope model can be better expressed as:
        // Ωmeas = T*K*(Ωtrue + b)
        // Ωmeas = M*(Ωtrue + b)
        // Ωmeas = M*Ωtrue + M*b

        // where:
        // M = I + Mg
        // bg = M*b = (I + Mg)*b --> b = M^-1*bg

        // We know that the norm of the true angular rate when the device is in a pixed
        // and unknown position and orientation is equal to the Earth rotation rate.
        // ||Ωtrue|| = 7.292115E-5 rad/s

        // Hence
        // Ωmeas - M*b = M*Ωtrue

        // M^-1 * (Ωmeas - M*b) = Ωtrue

        // ||Ωtrue||^2 = (M^-1 * (Ωmeas - M*b))^T*(M^-1 * (Ωmeas - M*b))
        // ||Ωtrue||^2 = (Ωmeas - M*b)^T * (M^-1)^T * M^-1 * (Ωmeas - M*b)
        // ||Ωtrue||^2 = (Ωmeas - M*b)^T * ||M^-1||^2 * (Ωmeas - M*b)
        // ||Ωtrue||^2 = ||Ωmeas - M*b||^2 * ||M^-1||^2

        // Where:

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11 	m12 	m13]
        //     [m21 	m22 	m23]
        //     [m31 	m32 	m33]

        final var gradientEstimator = new GradientEstimator(this::evaluateGeneral);

        final var initialM = Matrix.identity(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        initialM.add(getInitialMg());

        final var invInitialM = Utils.inverse(initialM);
        final var initialBg = getInitialBiasAsMatrix();
        final var initialB = invInitialM.multiplyAndReturnNew(initialBg);

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiDimensionFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are measured angular rate coordinates
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

                measAngularRateX = point[0];
                measAngularRateY = point[1];
                measAngularRateZ = point[2];

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

        final var mb = new Matrix(BodyKinematics.COMPONENTS, 1);
        mb.setElementAtIndex(0, bx);
        mb.setElementAtIndex(1, by);
        mb.setElementAtIndex(2, bz);

        final var mm = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
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

        // at this point covariance is expressed in terms of b, M and G, and must
        // be expressed in terms of bg, Mg and Gg.
        // We know that:
        // bg = M * b
        // Mg = M - I
        // Gg = M * G = 0

        // b = [bx]
        //     [by]
        //     [bz]

        // M = [m11  m12  m13]
        //     [m21  m22  m23]
        //     [m31  m32  m33]

        // G = [g11  g12  g13] = 0
        //     [g21  g22  g23]
        //     [g31  g32  g33]

        // bg = [m11  m12  m13][bx] = [m11 * bx + m12 * by + m13 * bz] = [bgx]
        //      [m21  m22  m23][by]   [m21 * bx + m22 * by + m23 * bz]   [bgy]
        //      [m31  m32  m33][bz]   [m31 * bx + m32 * by + m33 * bz]   [bgz]

        // Mg = [sx  mxy  mxz] = [m11 - 1    m12         m13     ]
        //      [myx sy	  myz]   [m21        m22 - 1     m23     ]
        //      [mzx mzy  sz ]   [m31        m32         m33 - 1 ]

        // Gg = [gg11  gg12  gg13] = 0
        //      [gg21  gg22  gg23]
        //      [gg31  gg32  gg33]

        // Defining the linear application:
        // F(b, M) = F(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33)
        // as:
        // [bgx] =  [m11 * bx + m12 * by + m13 * bz]
        // [bgy]    [m21 * bx + m22 * by + m23 * bz]
        // [bgz]    [m31 * bx + m32 * by + m33 * bz]
        // [sx]     [m11 - 1]
        // [sy]     [m22 - 1]
        // [sz]     [m33 - 1]
        // [mxy]    [m12]
        // [mxz]    [m13]
        // [myx]    [m21]
        // [myz]    [m23]
        // [mzx]    [m31]
        // [mzy]    [m32]
        // [gg11]   [0]
        // [gg21]   [0]
        // [gg31]   [0]
        // [gg12]   [0]
        // [gg22]   [0]
        // [gg32]   [0]
        // [gg13]   [0]
        // [gg23]   [0]
        // [gg33]   [0]

        // Then the Jacobian of F(b, M) is:
        // J = [m11  m12  m13  bx   0    0    by   0    0    bz   0    0 ]
        //     [m21  m22  m23  0    bx   0    0    by   0    0    bz   0 ]
        //     [m31  m32  m33  0    0    bx   0    0    by   0    0    bz]
        //     [0    0    0    1    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    1    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    1 ]
        //     [0    0    0    0    0    0    1    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    1    0    0 ]
        //     [0    0    0    0    1    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    1    0 ]
        //     [0    0    0    0    0    1    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    1    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]
        //     [0    0    0    0    0    0    0    0    0    0    0    0 ]

        // We know that the propagated covariance is J * Cov * J', hence:
        final var jacobian = new Matrix(GENERAL_UNKNOWNS_AND_CROSS_BIASES, GENERAL_UNKNOWNS);

        jacobian.setElementAt(0, 0, m11);
        jacobian.setElementAt(0, 1, m12);
        jacobian.setElementAt(0, 2, m13);
        jacobian.setElementAt(0, 3, bx);
        jacobian.setElementAt(0, 6, by);
        jacobian.setElementAt(0, 9, bz);

        jacobian.setElementAt(1, 0, m21);
        jacobian.setElementAt(1, 1, m22);
        jacobian.setElementAt(1, 2, m23);
        jacobian.setElementAt(1, 4, bx);
        jacobian.setElementAt(1, 7, by);
        jacobian.setElementAt(1, 10, bz);

        jacobian.setElementAt(2, 0, m31);
        jacobian.setElementAt(2, 1, m32);
        jacobian.setElementAt(2, 2, m33);
        jacobian.setElementAt(2, 5, bx);
        jacobian.setElementAt(2, 8, by);
        jacobian.setElementAt(2, 11, bz);

        jacobian.setElementAt(3, 3, 1.0);
        jacobian.setElementAt(4, 7, 1.0);
        jacobian.setElementAt(5, 11, 1.0);

        jacobian.setElementAt(6, 6, 1.0);
        jacobian.setElementAt(7, 9, 1.0);
        jacobian.setElementAt(8, 4, 1.0);

        jacobian.setElementAt(9, 10, 1.0);
        jacobian.setElementAt(10, 5, 1.0);
        jacobian.setElementAt(11, 8, 1.9);

        final var jacobianTrans = jacobian.transposeAndReturnNew();
        jacobian.multiply(estimatedCovariance);
        jacobian.multiply(jacobianTrans);
        estimatedCovariance = jacobian;
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter when G-dependent cross biases
     * are taken into account.
     *
     * @throws AlgebraException                              if provided accelerometer cross coupling
     *                                                       errors are not valid.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens
     */
    private void setInputDataWithGDependentCrossBiases() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        // compute reference frame at current position
        final var nedPosition = getNedPosition();
        final var nedC = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var refKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                ecefFrame);

        final var refAngularRateX = refKinematics.getAngularRateX();
        final var refAngularRateY = refKinematics.getAngularRateY();
        final var refAngularRateZ = refKinematics.getAngularRateZ();

        final var w2 = turntableRotationRate * turntableRotationRate;

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, 2 * BodyKinematics.COMPONENTS);
        final var y = new double[numMeasurements];
        final var angularRateStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredKinematics = measurement.getKinematics();

            final var angularRateX = measuredKinematics.getAngularRateX();
            final var angularRateY = measuredKinematics.getAngularRateY();
            final var angularRateZ = measuredKinematics.getAngularRateZ();

            final var fX = measuredKinematics.getFx();
            final var fY = measuredKinematics.getFy();
            final var fZ = measuredKinematics.getFz();

            x.setElementAt(i, 0, angularRateX - refAngularRateX);
            x.setElementAt(i, 1, angularRateY - refAngularRateY);
            x.setElementAt(i, 2, angularRateZ - refAngularRateZ);

            x.setElementAt(i, 3, fX);
            x.setElementAt(i, 4, fY);
            x.setElementAt(i, 5, fZ);

            y[i] = w2;

            angularRateStandardDeviations[i] = measurement.getAngularRateStandardDeviation();

            i++;
        }

        fitter.setInputData(x, y, angularRateStandardDeviations);

        ba = getAccelerometerBiasAsMatrix();
        ma = getAccelerometerMa();
        accelerationFixer.setBias(ba);
        accelerationFixer.setCrossCouplingErrors(ma);
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter when G-dependent cross biases
     * are ignored.
     *
     * @throws AlgebraException                              if provided accelerometer cross coupling
     *                                                       errors are not valid.
     * @throws InvalidSourceAndDestinationFrameTypeException never happens.
     */
    private void setInputData() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {

        // compute reference frame at current position
        final var nedPosition = getNedPosition();
        final var nedC = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var refKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(timeInterval, ecefFrame,
                ecefFrame);

        final var refAngularRateX = refKinematics.getAngularRateX();
        final var refAngularRateY = refKinematics.getAngularRateY();
        final var refAngularRateZ = refKinematics.getAngularRateZ();

        final var w2 = turntableRotationRate * turntableRotationRate;

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, BodyKinematics.COMPONENTS);
        final var y = new double[numMeasurements];
        final var angularRateStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredKinematics = measurement.getKinematics();

            final var angularRateX = measuredKinematics.getAngularRateX();
            final var angularRateY = measuredKinematics.getAngularRateY();
            final var angularRateZ = measuredKinematics.getAngularRateZ();

            x.setElementAt(i, 0, angularRateX - refAngularRateX);
            x.setElementAt(i, 1, angularRateY - refAngularRateY);
            x.setElementAt(i, 2, angularRateZ - refAngularRateZ);

            y[i] = w2;

            angularRateStandardDeviations[i] = measurement.getAngularRateStandardDeviation();

            i++;
        }

        fitter.setInputData(x, y, angularRateStandardDeviations);

        ba = getAccelerometerBiasAsMatrix();
        ma = getAccelerometerMa();
        accelerationFixer.setBias(ba);
        accelerationFixer.setCrossCouplingErrors(ma);
    }

    /**
     * Converts provided NED position expressed in terms of latitude, longitude and height respect
     * mean Earth surface, to position expressed in ECEF coordinates.
     *
     * @param position NED position to be converted.
     * @return converted position expressed in ECEF coordinates.
     */
    private static ECEFPosition convertPosition(final NEDPosition position) {
        final var velocity = new ECEFVelocity();
        final var result = new ECEFPosition();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                position.getLatitude(), position.getLongitude(), position.getHeight(), 0.0, 0.0, 0.0,
                result, velocity);
        return result;
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
     * Converts time instance to seconds.
     *
     * @param time time instance to be converted.
     * @return converted value.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Makes proper conversion of internal cross-coupling, bias and g-dependent
     * cross bias matrices.
     *
     * @param m internal scaling and cross-coupling matrix.
     * @param b internal bias matrix.
     * @param g internal g-dependent cross bias matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m, final Matrix b, final Matrix g) throws AlgebraException {
        setResult(m, b);

        // Gg = M*G
        m.multiply(g, estimatedGg);
    }

    /**
     * Makes proper conversion of internal cross-coupling and bias matrices.
     *
     * @param m internal scaling and cross-coupling matrix.
     * @param b internal bias matrix.
     * @throws AlgebraException if a numerical instability occurs.
     */
    private void setResult(final Matrix m, final Matrix b) throws AlgebraException {
        // Because:
        // M = I + Mg
        // b = M^-1*bg

        // Then:
        // Mg = M - I
        // bg = M*b

        if (estimatedBiases == null) {
            estimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        final var bg = m.multiplyAndReturnNew(b);
        bg.toArray(estimatedBiases);

        if (estimatedMg == null) {
            estimatedMg = m;
        } else {
            estimatedMg.copyFrom(m);
        }

        for (var i = 0; i < BodyKinematics.COMPONENTS; i++) {
            estimatedMg.setElementAt(i, i, estimatedMg.getElementAt(i, i) - 1.0);
        }

        if (estimatedGg == null) {
            estimatedGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        } else {
            estimatedGg.initialize(0.0);
        }

        estimatedCovariance = fitter.getCovar();
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and specific force along with provided parameters for the
     * general case when G-dependent cross biases are taken into account.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 21.
     * @return estimated true angular rate squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateGeneralWithGDependentCrossBiases(final double[] params) throws EvaluationException {
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

        final var g11 = params[12];
        final var g21 = params[13];
        final var g31 = params[14];

        final var g12 = params[15];
        final var g22 = params[16];
        final var g32 = params[17];

        final var g13 = params[18];
        final var g23 = params[19];
        final var g33 = params[20];

        return evaluate(bx, by, bz, m11, m21, m31, m12, m22, m32, m13, m23, m33,
                g11, g21, g31, g12, g22, g32, g13, g23, g33);
    }

    /**
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and specific force along with provided parameters when
     * common z-axis is assumed and G-dependent cross biases are taken into
     * account.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing parameters for the general purpose case
     *               when G-dependent cross biases are taken into account. Must
     *               have length 18.
     * @return estimated true angular rate squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluateCommonAxisWithGDependentCrossBiases(final double[] params) throws EvaluationException {
        final var bx = params[0];
        final var by = params[1];
        final var bz = params[2];

        final var m11 = params[3];

        final var m12 = params[4];
        final var m22 = params[5];

        final var m13 = params[6];
        final var m23 = params[7];
        final var m33 = params[8];

        final var g11 = params[9];
        final var g21 = params[10];
        final var g31 = params[11];

        final var g12 = params[12];
        final var g22 = params[13];
        final var g32 = params[14];

        final var g13 = params[15];
        final var g23 = params[16];
        final var g33 = params[17];

        return evaluate(bx, by, bz, m11, 0.0, 0.0, m12, m22, 0.0, m13, m23, m33,
                g11, g21, g31, g12, g22, g32, g13, g23, g33);
    }

    /**
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and provided parameters for the general case when G-dependent
     * cross biases are ignored.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the general purpose case
     *               when G-dependent cross biases are ignored. Must have length 12.
     * @return estimated true angular rate squared norm.
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
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and provided parameters when common z-axis is assumed and
     * G-dependent cross biases are ignored.
     * This method is internally executed during gradient estimation and
     * Levenberg-Marquardt fitting needed for calibration computation.
     *
     * @param params array containing current parameters for the common z-axis case
     *               when G-dependent cross biases are ignored. Must have length 9.
     * @return estimated true angular rate squared norm.
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
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and provided parameters.
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
     * @param g11 element 1,1 of g-dependent cross bias matrix.
     * @param g21 element 2,1 of g-dependent cross bias matrix.
     * @param g31 element 3,1 of g-dependent cross bias matrix.
     * @param g12 element 1,2 of g-dependent cross bias matrix.
     * @param g22 element 2,2 of g-dependent cross bias matrix.
     * @param g32 element 3,2 of g-dependent cross bias matrix.
     * @param g13 element 1,3 of g-dependent cross bias matrix.
     * @param g23 element 2,3 of g-dependent cross bias matrix.
     * @param g33 element 3,3 of g-dependent cross bias matrix.
     * @return estimated true angular rate squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluate(final double bx, final double by, final double bz,
                            final double m11, final double m21, final double m31,
                            final double m12, final double m22, final double m32,
                            final double m13, final double m23, final double m33,
                            final double g11, final double g21, final double g31,
                            final double g12, final double g22, final double g32,
                            final double g13, final double g23, final double g33) throws EvaluationException {

        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue
        // Ωmeas = M*(Ωtrue + b + G * ftrue)

        // M = I + Mg
        // bg = M*b --> b = M^-1*bg
        // Gg = M*G --> G = M^-1*Gg

        // Ωtrue = M^-1 * Ωmeas - b - G*ftrue

        try {
            if (measAngularRate == null) {
                measAngularRate = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
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
            if (g == null) {
                g = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }
            if (trueAngularRate == null) {
                trueAngularRate = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (ftrue == null) {
                ftrue = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (ba == null) {
                ba = new Matrix(BodyKinematics.COMPONENTS, 1);
            }
            if (ma == null) {
                ma = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
            }
            if (tmp == null) {
                tmp = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            measAngularRate.setElementAtIndex(0, measAngularRateX);
            measAngularRate.setElementAtIndex(1, measAngularRateY);
            measAngularRate.setElementAtIndex(2, measAngularRateZ);

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

            g.setElementAt(0, 0, g11);
            g.setElementAt(1, 0, g21);
            g.setElementAt(2, 0, g31);

            g.setElementAt(0, 1, g12);
            g.setElementAt(1, 1, g22);
            g.setElementAt(2, 1, g32);

            g.setElementAt(0, 2, g13);
            g.setElementAt(1, 2, g23);
            g.setElementAt(2, 2, g33);

            getAccelerometerBiasAsMatrix(ba);
            getAccelerometerMa(ma);

            // fix measured accelerometer value to obtain true
            // specific force
            accelerationFixer.fix(fmeas, ftrue);
            g.multiply(ftrue, tmp);

            invM.multiply(measAngularRate, trueAngularRate);
            trueAngularRate.subtract(b);
            trueAngularRate.subtract(tmp);

            final var norm = Utils.normF(trueAngularRate);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }

    /**
     * Computes estimated true angular rate squared norm using current measured
     * angular rate and provided parameters.
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
     * @return estimated true angular rate squared norm.
     * @throws EvaluationException if there are numerical instabilities.
     */
    private double evaluate(final double bx, final double by, final double bz,
                            final double m11, final double m21, final double m31,
                            final double m12, final double m22, final double m32,
                            final double m13, final double m23, final double m33) throws EvaluationException {

        // Ωmeas = M*(Ωtrue + b)
        // Ωtrue = M^-1 * Ωmeas - b

        try {
            if (measAngularRate == null) {
                measAngularRate = new Matrix(BodyKinematics.COMPONENTS, 1);
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
            if (trueAngularRate == null) {
                trueAngularRate = new Matrix(BodyKinematics.COMPONENTS, 1);
            }

            measAngularRate.setElementAtIndex(0, measAngularRateX);
            measAngularRate.setElementAtIndex(1, measAngularRateY);
            measAngularRate.setElementAtIndex(2, measAngularRateZ);

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

            invM.multiply(measAngularRate, trueAngularRate);
            trueAngularRate.subtract(b);

            final var norm = Utils.normF(trueAngularRate);
            return norm * norm;

        } catch (final AlgebraException e) {
            throw new EvaluationException(e);
        }
    }
}
