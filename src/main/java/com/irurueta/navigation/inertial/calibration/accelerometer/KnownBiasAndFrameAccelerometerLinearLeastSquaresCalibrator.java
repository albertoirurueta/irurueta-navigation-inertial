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
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationConverter;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Estimates accelerometer cross couplings and scaling factors.
 * This estimator assumes that biases are known.
 * <p>
 * This estimator uses a linear approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this estimator at least 4 measurements at different known frames must
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
 * - ba is accelerometer bias. This is a known 3x1 vector.
 * - I is the 3x3 identity matrix.
 * - Ma is the 3x3 matrix containing cross-couplings and scaling factors. Ideally, on
 * a perfect accelerometer, this should be a 3x3 zero matrix.
 * - ftrue is ground-truth specific force.
 * - w is measurement noise.
 */
@SuppressWarnings("DuplicatedCode")
public class KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator implements
        KnownBiasAndFrameAccelerometerCalibrator<FrameBodyKinematics,
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener>,
        UnorderedFrameBodyKinematicsAccelerometerCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 3;

    /**
     * Number of equations generated for each measurement.
     */
    private static final int EQUATIONS_PER_MEASUREMENT = 3;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 6;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 9;

    /**
     * Contains a collection of body kinematics measurements taken at different
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
     */
    private Collection<FrameBodyKinematics> measurements;

    /**
     * This flag indicates whether z-axis is assumed to be common for accelerometer
     * and gyroscope.
     * When enabled, this eliminates 3 variables from Ma matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this estimator.
     */
    private KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener;

    /**
     * Known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasX;

    /**
     * Known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasY;

    /**
     * Known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     */
    private double biasZ;

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
     * Indicates whether estimator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements) {
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        this(measurements);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasY known y coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     * @param biasZ known z coordinate of accelerometer bias expressed in meters per
     *              squared second (m/s^2).
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        try {
            setBiasCoordinates(biasX, biasY, biasZ);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasY    known y coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param biasZ    known z coordinate of accelerometer bias expressed in meters per
     *                 squared second (m/s^2).
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasY        known y coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param biasZ        known z coordinate of accelerometer bias expressed in meters
     *                     per squared second (m/s^2).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasY          known y coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param biasZ          known z coordinate of accelerometer bias expressed in
     *                       meters per squared second (m/s^2).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of accelerometer bias.
     * @param biasY known y coordinate of accelerometer bias.
     * @param biasZ known z coordinate of accelerometer bias.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
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
     * @param biasX    known x coordinate of accelerometer bias.
     * @param biasY    known y coordinate of accelerometer bias.
     * @param biasZ    known z coordinate of accelerometer bias.
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ) {
        this(biasX, biasY, biasZ);
        this.measurements = measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param biasX        known x coordinate of accelerometer bias.
     * @param biasY        known y coordinate of accelerometer bias.
     * @param biasZ        known z coordinate of accelerometer bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(biasX, biasY, biasZ, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ,
            final boolean commonAxisUsed) {
        this(measurements, biasX, biasY, biasZ);
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param measurements   collection of body kinematics measurements taken at
     *                       different frames (positions, orientations and velocities).
     * @param biasX          known x coordinate of accelerometer bias.
     * @param biasY          known y coordinate of accelerometer bias.
     * @param biasZ          known z coordinate of accelerometer bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<FrameBodyKinematics> measurements,
            final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ, final boolean commonAxisUsed,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, biasX, biasY, biasZ, commonAxisUsed);
        this.listener = listener;
    }

    /**
     * Gets a collection of body kinematics measurements taken at different
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
     * @return a collection of body kinematics measurements taken at different
     * frames (positions, orientations and velocities).
     */
    @Override
    public Collection<FrameBodyKinematics> getMeasurements() {
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
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setMeasurements(final Collection<? extends FrameBodyKinematics> measurements) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        //noinspection unchecked
        this.measurements = (Collection<FrameBodyKinematics>) measurements;
    }

    /**
     * Indicates the type of measurement used by this calibrator.
     *
     * @return type of measurement used by this calibrator.
     */
    @Override
    public AccelerometerCalibratorMeasurementType getMeasurementType() {
        return AccelerometerCalibratorMeasurementType.FRAME_BODY_KINEMATICS;
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
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    @Override
    public KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setListener(
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x coordinate of accelerometer bias.
     */
    @Override
    public double getBiasX() {
        return biasX;
    }

    /**
     * Sets known x coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasX(final double biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.biasX = biasX;
    }

    /**
     * Gets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y coordinate of accelerometer bias.
     */
    @Override
    public double getBiasY() {
        return biasY;
    }

    /**
     * Sets known y coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasY(final double biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasY = biasY;
    }

    /**
     * Gets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z coordinate of accelerometer bias.
     */
    @Override
    public double getBiasZ() {
        return biasZ;
    }

    /**
     * Sets known z coordinate of accelerometer bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasZ(final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasZ = biasZ;
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @return x coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasXAsAcceleration() {
        return new Acceleration(biasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known x coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasXAsAcceleration(final Acceleration result) {
        result.setValue(biasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known x coordinate of accelerometer bias.
     *
     * @param biasX x coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasX(final Acceleration biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasX = convertAcceleration(biasX);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @return y coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasYAsAcceleration() {
        return new Acceleration(biasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known y coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasYAsAcceleration(final Acceleration result) {
        result.setValue(biasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known y coordinate of accelerometer bias.
     *
     * @param biasY y coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasY(final Acceleration biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasY = convertAcceleration(biasY);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @return z coordinate of accelerometer bias.
     */
    @Override
    public Acceleration getBiasZAsAcceleration() {
        return new Acceleration(biasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets known z coordinate of accelerometer bias.
     *
     * @param result instance where result data will be stored.
     */
    @Override
    public void getBiasZAsAcceleration(final Acceleration result) {
        result.setValue(biasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets known z coordinate of accelerometer bias.
     *
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasZ(final Acceleration biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.biasZ = convertAcceleration(biasZ);
    }

    /**
     * Sets known accelerometer bias coordinates expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
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
     * Sets known accelerometer bias coordinates.
     *
     * @param biasX z coordinate of accelerometer bias.
     * @param biasY y coordinate of accelerometer bias.
     * @param biasZ z coordinate of accelerometer bias.
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void setBiasCoordinates(final Acceleration biasX, final Acceleration biasY, final Acceleration biasZ)
            throws LockedException {
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
     * Gets known accelerometer bias as an array.
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
     * Gets known accelerometer bias as an array.
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
     * Sets known accelerometer bias as an array.
     * Array values are expressed in meters per squared second (m/s^2).
     *
     * @param bias known accelerometer bias.
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
     * Gets known accelerometer bias as a column matrix.
     *
     * @return known accelerometer bias as a column matrix.
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
     * Gets known accelerometer bias as a column matrix.
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
     * Sets known accelerometer bias as a column matrix.
     *
     * @param bias accelerometer bias to be set.
     * @throws LockedException          if calibrator is currently running
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
    public int getMinimumRequiredMeasurements() {
        return MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether estimator is ready to start the estimator.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return measurements != null && measurements.size() >= MINIMUM_MEASUREMENTS;
    }

    /**
     * Indicates whether estimator is currently running or not.
     *
     * @return true if estimator is running, false otherwise.
     */
    @Override
    public boolean isRunning() {
        return running;
    }

    /**
     * Estimates accelerometer calibration parameters containing scale factors
     * and cross-coupling errors.
     *
     * @throws LockedException      if estimator is currently running.
     * @throws NotReadyException    if estimator is not ready.
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

        } catch (final AlgebraException e) {
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateCommonAxis() throws AlgebraException {
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

        // Where the unknowns are: sx, sy, sz, mxy mxz, myz
        // Reordering:
        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + ftruez + sz * ftruez

        // fmeasx - ftruex - bx = sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy - ftruey - by = sy * ftruey + myz * ftruez
        // fmeasz - ftruez - bz = sz * ftruez

        // [ftruex  0       0       ftruey  ftruez  0     ][sx ] = [fmeasx - ftruex - bx]
        // [0       ftruey  0       0       0       ftruez][sy ]   [fmeasy - ftruey - by]
        // [0       0       ftruez  0       0       0     ][sz ]   [fmeasz - ftruez - bz]
        //                                                 [mxy]
        //                                                 [mxz]
        //                                                 [myz]

        final var expectedKinematics = new BodyKinematics();

        final var rows = EQUATIONS_PER_MEASUREMENT * measurements.size();
        final var a = new Matrix(rows, COMMON_Z_AXIS_UNKNOWNS);
        final var b = new Matrix(rows, 1);
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

            a.setElementAt(i, 0, fTrueX);
            a.setElementAt(i, 1, 0.0);
            a.setElementAt(i, 2, 0.0);
            a.setElementAt(i, 3, fTrueY);
            a.setElementAt(i, 4, fTrueZ);
            a.setElementAt(i, 5, 0.0);

            b.setElementAtIndex(i, fMeasX - fTrueX - biasX);
            i++;

            a.setElementAt(i, 0, 0.0);
            a.setElementAt(i, 1, fTrueY);
            a.setElementAt(i, 2, 0.0);
            a.setElementAt(i, 3, 0.0);
            a.setElementAt(i, 4, 0.0);
            a.setElementAt(i, 5, fTrueZ);

            b.setElementAtIndex(i, fMeasY - fTrueY - biasY);
            i++;

            a.setElementAt(i, 0, 0.0);
            a.setElementAt(i, 1, 0.0);
            a.setElementAt(i, 2, fTrueZ);
            a.setElementAt(i, 3, 0.0);
            a.setElementAt(i, 4, 0.0);
            a.setElementAt(i, 5, 0.0);

            b.setElementAtIndex(i, fMeasZ - fTrueZ - biasZ);
            i++;
        }

        final var unknowns = Utils.solve(a, b);

        final var sx = unknowns.getElementAtIndex(0);
        final var sy = unknowns.getElementAtIndex(1);
        final var sz = unknowns.getElementAtIndex(2);
        final var mxy = unknowns.getElementAtIndex(3);
        final var mxz = unknowns.getElementAtIndex(4);
        final var myz = unknowns.getElementAtIndex(5);

        fillMa(sx, sy, sz, mxy, mxz, 0.0, myz, 0.0, 0.0);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateGeneral() throws AlgebraException {
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

        // Where the unknowns are: sx, sy, sz, mxy mxz, myx, myz, mzx, mzy
        // Reordering:
        // fmeasx = bx + ftruex + sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy = by + myx * ftruex + ftruey + sy * ftruey + myz * ftruez
        // fmeasz = bz + mzx * ftruex + mzy * ftruey + ftruez + sz * ftruez

        // fmeasx - ftruex - bx = sx * ftruex + mxy * ftruey + mxz * ftruez
        // fmeasy - ftruey - by = myx * ftruex + sy * ftruey + myz * ftruez
        // fmeasz - ftruez - bz = mzx * ftruex + mzy * ftruey + sz * ftruez

        // [ftruex  0       0       ftruey  ftruez  0       0       0       0     ][sx ] = [fmeasx - ftruex - bx]
        // [0       ftruey  0       0       0       ftruex  ftruez  0       0     ][sy ]   [fmeasy - ftruey - by]
        // [0       0       ftruez  0       0       0       0       ftruex  ftruey][sz ]   [fmeasz - ftruez - bz]
        //                                                                         [mxy]
        //                                                                         [mxz]
        //                                                                         [myx]
        //                                                                         [myz]
        //                                                                         [mzx]
        //                                                                         [mzy]

        final var expectedKinematics = new BodyKinematics();

        final var rows = EQUATIONS_PER_MEASUREMENT * measurements.size();
        final var a = new Matrix(rows, GENERAL_UNKNOWNS);
        final var b = new Matrix(rows, 1);
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

            a.setElementAt(i, 0, fTrueX);
            a.setElementAt(i, 1, 0.0);
            a.setElementAt(i, 2, 0.0);
            a.setElementAt(i, 3, fTrueY);
            a.setElementAt(i, 4, fTrueZ);
            a.setElementAt(i, 5, 0.0);
            a.setElementAt(i, 6, 0.0);
            a.setElementAt(i, 7, 0.0);
            a.setElementAt(i, 8, 0.0);

            b.setElementAtIndex(i, fMeasX - fTrueX - biasX);
            i++;

            a.setElementAt(i, 0, 0.0);
            a.setElementAt(i, 1, fTrueY);
            a.setElementAt(i, 2, 0.0);
            a.setElementAt(i, 3, 0.0);
            a.setElementAt(i, 4, 0.0);
            a.setElementAt(i, 5, fTrueX);
            a.setElementAt(i, 6, fTrueZ);
            a.setElementAt(i, 7, 0.0);
            a.setElementAt(i, 8, 0.0);

            b.setElementAtIndex(i, fMeasY - fTrueY - biasY);
            i++;

            a.setElementAt(i, 0, 0.0);
            a.setElementAt(i, 1, 0.0);
            a.setElementAt(i, 2, fTrueZ);
            a.setElementAt(i, 3, 0.0);
            a.setElementAt(i, 4, 0.0);
            a.setElementAt(i, 5, 0.0);
            a.setElementAt(i, 6, 0.0);
            a.setElementAt(i, 7, fTrueX);
            a.setElementAt(i, 8, fTrueY);

            b.setElementAtIndex(i, fMeasZ - fTrueZ - biasZ);
            i++;
        }

        final var unknowns = Utils.solve(a, b);

        final var sx = unknowns.getElementAtIndex(0);
        final var sy = unknowns.getElementAtIndex(1);
        final var sz = unknowns.getElementAtIndex(2);
        final var mxy = unknowns.getElementAtIndex(3);
        final var mxz = unknowns.getElementAtIndex(4);
        final var myx = unknowns.getElementAtIndex(5);
        final var myz = unknowns.getElementAtIndex(6);
        final var mzx = unknowns.getElementAtIndex(7);
        final var mzy = unknowns.getElementAtIndex(8);

        fillMa(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Fills scale factor and cross coupling error matrix with estimated values.
     *
     * @param sx  x scale factor
     * @param sy  y scale factor
     * @param sz  z scale factor
     * @param mxy x-y cross coupling
     * @param mxz x-z cross coupling
     * @param myx y-x cross coupling
     * @param myz y-z cross coupling
     * @param mzx z-x cross coupling
     * @param mzy z-y cross coupling
     * @throws WrongSizeException never happens.
     */
    private void fillMa(final double sx, final double sy, final double sz,
                        final double mxy, final double mxz, final double myx,
                        final double myz, final double mzx, final double mzy) throws WrongSizeException {
        if (estimatedMa == null) {
            estimatedMa = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
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
