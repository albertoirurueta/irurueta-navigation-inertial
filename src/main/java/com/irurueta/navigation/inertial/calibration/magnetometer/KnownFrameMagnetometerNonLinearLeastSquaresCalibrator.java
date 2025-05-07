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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
import com.irurueta.numerical.fitting.FittingException;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFitter;
import com.irurueta.numerical.fitting.LevenbergMarquardtMultiVariateFunctionEvaluator;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityConverter;
import com.irurueta.units.MagneticFluxDensityUnit;

import java.io.IOException;
import java.util.Collection;

/**
 * Estimates magnetometer hard-iron biases, cross couplings and scaling
 * factors.
 * <p>
 * This calibrator uses an iterative approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 4 measurements at different known frames must
 * be provided. In other words, magnetometer samples must be obtained at 4
 * different positions or orientations.
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
public class KnownFrameMagnetometerNonLinearLeastSquaresCalibrator implements
        KnownFrameMagnetometerCalibrator<StandardDeviationFrameBodyMagneticFluxDensity,
                KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener>, MagnetometerNonLinearCalibrator,
        UnknownHardIronNonLinearMagnetometerCalibrator,
        UnorderedStandardDeviationFrameBodyMagneticFluxDensityMagnetometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for the accelerometer,
     * gyroscope and magnetometer.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 4;

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
     * Levenberg-Marquardt fitter to find a non-linear solution.
     */
    private final LevenbergMarquardtMultiVariateFitter fitter = new LevenbergMarquardtMultiVariateFitter();

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
    private Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer,
     * gyroscope and magnetometer.
     * When enabled, this eliminates 3 variables from Mm matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener;

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
     * Contains Earth's magnetic model.
     */
    private WorldMagneticModel magneticModel;

    /**
     * Constructor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body magnetic flux density measurements with
     *                     standard deviations taken at different frames (positions
     *                     and orientations).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(final WorldMagneticModel magneticModel) {
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(listener);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel) {
        this(measurements);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements  collection of body magnetic flux density measurements with
     *                      standard deviations taken at different frames (positions
     *                      and orientations).
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @param listener      listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, listener);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel) {
        this(commonAxisUsed);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, listener);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel) {
        this(measurements, commonAxisUsed);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body magnetic flux density measurements with
     *                       standard deviations taken at different frames (positions
     *                       and orientations).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel  Earth's magnetic model. If null, a default model
     *                       will be used instead.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, listener);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ) {
        try {
            setInitialHardIron(initialHardIronX, initialHardIronY, initialHardIronZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        this.measurements = measurements;
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double initialHardIronX,
            final double initialHardIronY, final double initialHardIronZ) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double initialHardIronX,
            final double initialHardIronY, final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
        try {
            setInitialScalingFactors(initialSx, initialSy, initialSz);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ);
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
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ, initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx, initialMyz, initialMzx, initialMzy);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel,
            final double initialHardIronX, final double initialHardIronY, final double initialHardIronZ,
            final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy) {
        this(commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements     collection of body magnetic flux density measurements with
     *                         standard deviations taken at different frames (positions
     *                         and orientations).
     * @param commonAxisUsed   indicates whether z-axis is assumed to be common
     *                         for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel    Earth's magnetic model. If null, a default model
     *                         will be used instead.
     * @param initialHardIronX initial x-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronY initial y-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialHardIronZ initial z-coordinate of magnetometer
     *                         hard-iron bias expressed in Teslas (T).
     * @param initialSx        initial x scaling factor.
     * @param initialSy        initial y scaling factor.
     * @param initialSz        initial z scaling factor.
     * @param initialMxy       initial x-y cross coupling error.
     * @param initialMxz       initial x-z cross coupling error.
     * @param initialMyx       initial y-x cross coupling error.
     * @param initialMyz       initial y-z cross coupling error.
     * @param initialMzx       initial z-x cross coupling error.
     * @param initialMzy       initial z-y cross coupling error.
     * @param listener         listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final double initialHardIronX, final double initialHardIronY,
            final double initialHardIronZ, final double initialSx, final double initialSy, final double initialSz,
            final double initialMxy, final double initialMxz, final double initialMyx,
            final double initialMyz, final double initialMzx, final double initialMzy,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIronX, initialHardIronY, initialHardIronZ,
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(final double[] initialHardIron) {
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
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron) {
        this(initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron) {
        this(commonAxisUsed, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double[] initialHardIron) {
        this(initialHardIron);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double[] initialHardIron) {
        this(magneticModel, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double[] initialHardIron) {
        this(magneticModel, initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double[] initialHardIron) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final double[] initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(final Matrix initialHardIron) {
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
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        this(initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron) {
        this(commonAxisUsed, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final Matrix initialHardIron) {
        this(initialHardIron);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron) {
        this(magneticModel, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final Matrix initialHardIron) {
        this(magneticModel, initialHardIron);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final Matrix initialHardIron) {
        this(commonAxisUsed, magneticModel, initialHardIron);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron);
        this.listener = listener;
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
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(final Matrix initialHardIron, final Matrix initialMm) {
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
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this
     *                        calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
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
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm) {
        this(commonAxisUsed, initialHardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
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
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm) {
        this(initialHardIron, initialMm);
        this.magneticModel = magneticModel;
    }

    /**
     * Constructor.
     *
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(magneticModel, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm) {
        this(magneticModel, initialHardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, magneticModel, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final Matrix initialMm) {
        this(magneticModel, initialHardIron, initialMm);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final WorldMagneticModel magneticModel, final Matrix initialHardIron,
            final Matrix initialMm, final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed, magneticModel, initialHardIron, initialMm);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm) {
        this(commonAxisUsed, magneticModel, initialHardIron, initialMm);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements    collection of body magnetic flux density measurements with
     *                        standard deviations taken at different frames (positions
     *                        and orientations).
     * @param commonAxisUsed  indicates whether z-axis is assumed to be common
     *                        for the accelerometer, gyroscope and magnetometer.
     * @param magneticModel   Earth's magnetic model. If null, a default model
     *                        will be used instead.
     * @param initialHardIron initial hard-iron to find a solution.
     * @param initialMm       initial soft-iron matrix containing scale factors
     *                        and cross coupling errors.
     * @param listener        listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
            final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final WorldMagneticModel magneticModel, final Matrix initialHardIron, final Matrix initialMm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed, magneticModel, initialHardIron, initialMm);
        this.listener = listener;
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
     * Gets initial hard-iron bias to be used to find a solution as an array.
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
     * Gets a collection of body magnetic flux density measurements taken at different
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
    public Collection<StandardDeviationFrameBodyMagneticFluxDensity> getMeasurements() {
        return measurements;
    }

    /**
     * Sets a collection of body magnetic flux density measurements taken at different
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
    public void setMeasurements(final Collection<? extends StandardDeviationFrameBodyMagneticFluxDensity> measurements)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }
        //noinspection unchecked
        this.measurements = (Collection<StandardDeviationFrameBodyMagneticFluxDensity>) measurements;
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
    @Override
    public KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(final KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener listener)
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

        } catch (final AlgebraException | FittingException | com.irurueta.numerical.NotReadyException | IOException e) {
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
            result.setValueCoordinatesAndUnit(
                    estimatedHardIron[0], estimatedHardIron[1], estimatedHardIron[2], MagneticFluxDensityUnit.TESLA);
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
    public boolean getEstimatedHardIronXStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
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
    public boolean getEstimatedHardIronYStandardDeviationAsMagneticFluxDensity(final MagneticFluxDensity result) {
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
                + getEstimatedHardIronZStandardDeviation()) / 3.0 : null;
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
                + getEstimatedHardIronZVariance())
                : null;
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
    public boolean getEstimatedHardIronStandardDeviationNormAsMagneticFluxDensity(final MagneticFluxDensity result) {
        if (estimatedCovariance != null) {
            result.setValue(getEstimatedHardIronStandardDeviationNorm());
            result.setUnit(MagneticFluxDensityUnit.TESLA);
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
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateCommonAxis() throws AlgebraException, FittingException,
            com.irurueta.numerical.NotReadyException, IOException {
        // The accelerometer model is:
        // mBmeas = ba + (I + Ma) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = ba + (I + Ma) * mBtrue

        // Hence:
        // [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        // [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        // [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        // where myx = mzx = mzy = 0

        // Hence:
        // [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        // [mBmeasy] = [by]     [0  1   0]   [0     sy  myz]    [mBtruey]
        // [mBmeasz] = [bz]     [0  0   1]   [0     0   sz ]    [mBtruez]

        // [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        // [mBmeasy]   [by]     [0      1+sy    myz ][mBtruey]
        // [mBmeasz]   [bz]     [0      0       1+sz][mBtruez]

        // mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy = by + (1+sy) * mBtruey + myz * mBtruez
        // mBmeasz = bz + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz
        // Reordering:
        // mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
        // mBmeasz = bz + mBtruez + sz * mBtruez

        // mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy - mBtruey = by + sy * mBtruey + myz * mBtruez
        // mBmeasz - mBtruez = bz + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruez][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0      ][bz ]   [mBmeasz - mBtruez]
        //                                                                   [sx ]
        //                                                                   [sy ]
        //                                                                   [sz ]
        //                                                                   [mxy]
        //                                                                   [mxz]
        //                                                                   [myz]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true magnetic flux density coordinates
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured magnetic flux density
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[COMMON_Z_AXIS_UNKNOWNS];

                initial[0] = initialHardIronX;
                initial[1] = initialHardIronY;
                initial[2] = initialHardIronZ;

                initial[3] = initialSx;
                initial[4] = initialSy;
                initial[5] = initialSz;

                initial[6] = initialMxy;
                initial[7] = initialMxz;
                initial[8] = initialMyz;

                return initial;
            }

            @Override
            public void evaluate(
                    final int i, final double[] point, final double[] result, final double[] params,
                    final Matrix jacobian) {
                // We know that:
                // mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                // mBmeasy = by + mBtruey + sy * mBtruey + myz * mBtruez
                // mBmeasz = bz + mBtruez + sz * mBtruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myz

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = mBtruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = mBtruey
                // d(fmeasx)/d(mxz) = mBtruez
                // d(fmeasx)/d(myz) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = mBtruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myz) = mBtruez

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = mBtruez
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

                final var btruex = point[0];
                final var btruey = point[1];
                final var btruez = point[2];

                result[0] = bx + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                result[1] = by + btruey + sy * btruey + myz * btruez;
                result[2] = bz + btruez + sz * btruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, btruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, btruey);
                jacobian.setElementAt(0, 7, btruez);
                jacobian.setElementAt(0, 8, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, btruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, btruez);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, btruez);
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

        if (estimatedHardIron == null) {
            estimatedHardIron = new double[BodyMagneticFluxDensity.COMPONENTS];
        }

        estimatedHardIron[0] = bx;
        estimatedHardIron[1] = by;
        estimatedHardIron[2] = bz;

        if (estimatedMm == null) {
            estimatedMm = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        } else {
            estimatedMm.initialize(0.0);
        }

        estimatedMm.setElementAt(0, 0, sx);

        estimatedMm.setElementAt(0, 1, mxy);
        estimatedMm.setElementAt(1, 1, sy);

        estimatedMm.setElementAt(0, 2, mxz);
        estimatedMm.setElementAt(1, 2, myz);
        estimatedMm.setElementAt(2, 2, sz);

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
     * @throws IOException                              if world magnetic model cannot be loaded.
     */
    private void calibrateGeneral() throws AlgebraException, FittingException, com.irurueta.numerical.NotReadyException,
            IOException {
        // The magnetometer model is:
        // mBmeas = ba + (I + Mm) * mBtrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // mBmeas = ba + (I + Mm) * mBtrue

        // Hence:
        // [mBmeasx] = [bx] + ( [1  0   0] + [sx    mxy mxz])   [mBtruex]
        // [mBmeasy] = [by]     [0  1   0]   [myx   sy  myz]    [mBtruey]
        // [mBmeasz] = [bz]     [0  0   1]   [mzx   mzy sz ]    [mBtruez]

        // [mBmeasx] = [bx] +   [1+sx   mxy     mxz ][mBtruex]
        // [mBmeasy]   [by]     [myx    1+sy    myz ][mBtruey]
        // [mBmeasz]   [bz]     [mzx    mzy     1+sz][mBtruez]

        // mBmeasx = bx + (1+sx) * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy = by + myx * mBtruex + (1+sy) * mBtruey + myz * mBtruez
        // mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + (1+sz) * mBtruez

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        // mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
        // mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

        // mBmeasx - mBtruex = bx + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy - mBtruey = by + myx * mBtruex + sy * mBtruey + myz * mBtruez
        // mBmeasz - mBtruez = bz + mzx * mBtruex + mzy * mBtruey + sz * mBtruez

        // [1   0   0   mBtruex  0        0        mBtruey  mBtruez  0        0        0        0      ][bx ] = [mBmeasx - mBtruex]
        // [0   1   0   0        mBtruey  0        0        0        mBtruex  mBtruez  0        0      ][by ]   [mBmeasy - mBtruey]
        // [0   0   1   0        0        mBtruez  0        0        0        0        mBtruex  mBtruey][bz ]   [mBmeasz - mBtruez]
        //                                                                                              [sx ]
        //                                                                                              [sy ]
        //                                                                                              [sz ]
        //                                                                                              [mxy]
        //                                                                                              [mxz]
        //                                                                                              [myx]
        //                                                                                              [myz]
        //                                                                                              [mzx]
        //                                                                                              [mzy]

        fitter.setFunctionEvaluator(new LevenbergMarquardtMultiVariateFunctionEvaluator() {
            @Override
            public int getNumberOfDimensions() {
                // Input points are true magnetic flux density coordinates
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public int getNumberOfVariables() {
                // The multivariate function returns the components of measured magnetic flux density
                return BodyMagneticFluxDensity.COMPONENTS;
            }

            @Override
            public double[] createInitialParametersArray() {
                final var initial = new double[GENERAL_UNKNOWNS];

                initial[0] = initialHardIronX;
                initial[1] = initialHardIronY;
                initial[2] = initialHardIronZ;

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
            public void evaluate(
                    final int i, final double[] point, final double[] result, final double[] params,
                    final Matrix jacobian) {
                // We know that:
                // mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
                // mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
                // mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

                // Hence, the derivatives respect the parameters bx, by, bz, sx, sy,
                // sz, mxy, mxz, myx, myz, mzx and mzy is:

                // d(fmeasx)/d(bx) = 1.0
                // d(fmeasx)/d(by) = 0.0
                // d(fmeasx)/d(bz) = 0.0
                // d(fmeasx)/d(sx) = mBtruex
                // d(fmeasx)/d(sy) = 0.0
                // d(fmeasx)/d(sz) = 0.0
                // d(fmeasx)/d(mxy) = mBtruey
                // d(fmeasx)/d(mxz) = mBtruez
                // d(fmeasx)/d(myx) = 0.0
                // d(fmeasx)/d(myz) = 0.0
                // d(fmeasx)/d(mzx) = 0.0
                // d(fmeasx)/d(mzy) = 0.0

                // d(fmeasy)/d(bx) = 0.0
                // d(fmeasy)/d(by) = 1.0
                // d(fmeasy)/d(bz) = 0.0
                // d(fmeasy)/d(sx) = 0.0
                // d(fmeasy)/d(sy) = mBtruey
                // d(fmeasy)/d(sz) = 0.0
                // d(fmeasy)/d(mxy) = 0.0
                // d(fmeasy)/d(mxz) = 0.0
                // d(fmeasy)/d(myx) = mBtruex
                // d(fmeasy)/d(myz) = mBtruez
                // d(fmeasy)/d(mzx) = 0.0
                // d(fmeasy)/d(mzy) = 0.0

                // d(fmeasz)/d(bx) = 0.0
                // d(fmeasz)/d(by) = 0.0
                // d(fmeasz)/d(bz) = 1.0
                // d(fmeasz)/d(sx) = 0.0
                // d(fmeasz)/d(sy) = 0.0
                // d(fmeasz)/d(sz) = mBtruez
                // d(fmeasz)/d(mxy) = 0.0
                // d(fmeasz)/d(mxz) = 0.0
                // d(fmeasz)/d(myx) = 0.0
                // d(fmeasz)/d(myz) = 0.0
                // d(fmeasz)/d(mzx) = mBtruex
                // d(fmeasz)/d(mzy) = mBtruey

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

                final var btruex = point[0];
                final var btruey = point[1];
                final var btruez = point[2];

                result[0] = bx + btruex + sx * btruex + mxy * btruey + mxz * btruez;
                result[1] = by + myx * btruex + btruey + sy * btruey + myz * btruez;
                result[2] = bz + mzx * btruex + mzy * btruey + btruez + sz * btruez;

                jacobian.setElementAt(0, 0, 1.0);
                jacobian.setElementAt(0, 1, 0.0);
                jacobian.setElementAt(0, 2, 0.0);
                jacobian.setElementAt(0, 3, btruex);
                jacobian.setElementAt(0, 4, 0.0);
                jacobian.setElementAt(0, 5, 0.0);
                jacobian.setElementAt(0, 6, btruey);
                jacobian.setElementAt(0, 7, btruez);
                jacobian.setElementAt(0, 8, 0.0);
                jacobian.setElementAt(0, 9, 0.0);
                jacobian.setElementAt(0, 10, 0.0);
                jacobian.setElementAt(0, 11, 0.0);

                jacobian.setElementAt(1, 0, 0.0);
                jacobian.setElementAt(1, 1, 1.0);
                jacobian.setElementAt(1, 2, 0.0);
                jacobian.setElementAt(1, 3, 0.0);
                jacobian.setElementAt(1, 4, btruey);
                jacobian.setElementAt(1, 5, 0.0);
                jacobian.setElementAt(1, 6, 0.0);
                jacobian.setElementAt(1, 7, 0.0);
                jacobian.setElementAt(1, 8, btruex);
                jacobian.setElementAt(1, 9, btruez);
                jacobian.setElementAt(1, 10, 0.0);
                jacobian.setElementAt(1, 11, 0.0);

                jacobian.setElementAt(2, 0, 0.0);
                jacobian.setElementAt(2, 1, 0.0);
                jacobian.setElementAt(2, 2, 1.0);
                jacobian.setElementAt(2, 3, 0.0);
                jacobian.setElementAt(2, 4, 0.0);
                jacobian.setElementAt(2, 5, btruez);
                jacobian.setElementAt(2, 6, 0.0);
                jacobian.setElementAt(2, 7, 0.0);
                jacobian.setElementAt(2, 8, 0.0);
                jacobian.setElementAt(2, 9, 0.0);
                jacobian.setElementAt(2, 10, btruex);
                jacobian.setElementAt(2, 11, btruey);
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

        if (estimatedHardIron == null) {
            estimatedHardIron = new double[BodyMagneticFluxDensity.COMPONENTS];
        }

        estimatedHardIron[0] = bx;
        estimatedHardIron[1] = by;
        estimatedHardIron[2] = bz;

        if (estimatedMm == null) {
            estimatedMm = new Matrix(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
        } else {
            estimatedMm.initialize(0.0);
        }

        estimatedMm.setElementAt(0, 0, sx);
        estimatedMm.setElementAt(1, 0, myx);
        estimatedMm.setElementAt(2, 0, mzx);

        estimatedMm.setElementAt(0, 1, mxy);
        estimatedMm.setElementAt(1, 1, sy);
        estimatedMm.setElementAt(2, 1, mzy);

        estimatedMm.setElementAt(0, 2, mxz);
        estimatedMm.setElementAt(1, 2, myz);
        estimatedMm.setElementAt(2, 2, sz);

        estimatedCovariance = fitter.getCovar();
        estimatedChiSq = fitter.getChisq();
        estimatedMse = fitter.getMse();
    }

    /**
     * Sets input data into Levenberg-Marquardt fitter.
     *
     * @throws WrongSizeException never happens.
     * @throws IOException        if world magnetic model cannot be loaded.
     */
    private void setInputData() throws WrongSizeException, IOException {
        // set input data using:
        // mBmeasx = bx + mBtruex + sx * mBtruex + mxy * mBtruey + mxz * mBtruez
        // mBmeasy = by + myx * mBtruex + mBtruey + sy * mBtruey + myz * mBtruez
        // mBmeasz = bz + mzx * mBtruex + mzy * mBtruey + mBtruez + sz * mBtruez

        final WMMEarthMagneticFluxDensityEstimator wmmEstimator;
        if (magneticModel != null) {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator(magneticModel);
        } else {
            wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        }

        final var expectedMagneticFluxDensity = new BodyMagneticFluxDensity();
        final var nedFrame = new NEDFrame();
        final var earthB = new NEDMagneticFluxDensity();
        final var cbn = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        final var cnb = new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);

        final var numMeasurements = measurements.size();
        final var x = new Matrix(numMeasurements, BodyMagneticFluxDensity.COMPONENTS);
        final var y = new Matrix(numMeasurements, BodyMagneticFluxDensity.COMPONENTS);
        final var specificForceStandardDeviations = new double[numMeasurements];
        var i = 0;
        for (final var measurement : measurements) {
            final var measuredMagneticFluxDensity = measurement.getMagneticFluxDensity();

            // estimate Earth magnetic flux density at frame position and
            // timestamp using WMM
            final var ecefFrame = measurement.getFrame();
            ECEFtoNEDFrameConverter.convertECEFtoNED(ecefFrame, nedFrame);

            final var year = measurement.getYear();

            final var latitude = nedFrame.getLatitude();
            final var longitude = nedFrame.getLongitude();
            final var height = nedFrame.getHeight();

            nedFrame.getCoordinateTransformation(cbn);
            cbn.inverse(cnb);

            wmmEstimator.estimate(latitude, longitude, height, year, earthB);

            // estimate expected body magnetic flux density taking into
            // account body attitude (inverse of frame orientation) and
            // estimated Earth magnetic flux density
            BodyMagneticFluxDensityEstimator.estimate(earthB, cnb, expectedMagneticFluxDensity);

            final var bMeasX = measuredMagneticFluxDensity.getBx();
            final var bMeasY = measuredMagneticFluxDensity.getBy();
            final var bMeasZ = measuredMagneticFluxDensity.getBz();

            final var bTrueX = expectedMagneticFluxDensity.getBx();
            final var bTrueY = expectedMagneticFluxDensity.getBy();
            final var bTrueZ = expectedMagneticFluxDensity.getBz();

            x.setElementAt(i, 0, bTrueX);
            x.setElementAt(i, 1, bTrueY);
            x.setElementAt(i, 2, bTrueZ);

            y.setElementAt(i, 0, bMeasX);
            y.setElementAt(i, 1, bMeasY);
            y.setElementAt(i, 2, bMeasZ);

            specificForceStandardDeviations[i] = measurement.getMagneticFluxDensityStandardDeviation();
            i++;
        }

        fitter.setInputData(x, y, specificForceStandardDeviations);
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
