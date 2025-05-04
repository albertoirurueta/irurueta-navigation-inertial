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
import com.irurueta.navigation.inertial.calibration.AccelerometerCalibrationSource;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;

import java.util.Collection;

/**
 * Estimates accelerometer biases, cross couplings and scaling factors.
 * <p>
 * This calibrator uses a linear approach to find a minimum least squared error
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
 * - ftrue is ground-truth specific force. This is a 3x1 vector.
 * - w is measurement noise. This is a 3x1 vector.
 */
@SuppressWarnings("DuplicatedCode")
public class KnownFrameAccelerometerLinearLeastSquaresCalibrator implements
        KnownFrameAccelerometerCalibrator<FrameBodyKinematics,
                KnownFrameAccelerometerLinearLeastSquaresCalibratorListener>, AccelerometerCalibrationSource,
        UnorderedFrameBodyKinematicsAccelerometerCalibrator {

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
     * Number of equations generated for each measurement.
     */
    private static final int EQUATIONS_PER_MEASUREMENT = 3;

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
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener;

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
     * Indicates whether calibrator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements) {
        //noinspection unchecked
        this.measurements = (Collection<FrameBodyKinematics>) measurements;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements,
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements, final boolean commonAxisUsed) {
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) {
        this(measurements, commonAxisUsed);
        this.listener = listener;
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
     * @throws LockedException if calibrator is currently running.
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
    public KnownFrameAccelerometerLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(
            final KnownFrameAccelerometerLinearLeastSquaresCalibratorListener listener) throws LockedException {
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
     * Estimates accelerometer calibration parameters containing bias, scale factors
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

        } catch (final AlgebraException e) {
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
     * Gets estimated accelerometer scale factors and cross coupling errors.
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

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, fTrueX);
            a.setElementAt(i, 6, fTrueY);
            a.setElementAt(i, 7, fTrueZ);

            b.setElementAtIndex(i, fMeasX - fTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, fTrueY);
            a.setElementAt(i, 8, fTrueZ);

            b.setElementAtIndex(i, fMeasY - fTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, fTrueZ);

            b.setElementAtIndex(i, fMeasZ - fTrueZ);
            i++;
        }

        final var unknowns = Utils.solve(a, b);

        final var bx = unknowns.getElementAtIndex(0);
        final var by = unknowns.getElementAtIndex(1);
        final var bz = unknowns.getElementAtIndex(2);
        final var sx = unknowns.getElementAtIndex(3);
        final var sy = unknowns.getElementAtIndex(4);
        final var sz = unknowns.getElementAtIndex(5);
        final var mxy = unknowns.getElementAtIndex(6);
        final var mxz = unknowns.getElementAtIndex(7);
        final var myz = unknowns.getElementAtIndex(8);

        fillBiases(bx, by, bz);
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
        // [fmeasy]   [by]     [0  1   0]   [myx   sy  myz]    [ftruey]
        // [fmeasz]   [bz]     [0  0   1]   [mzx   mzy sz ]    [ftruez]

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

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, fTrueX);
            a.setElementAt(i, 6, fTrueY);
            a.setElementAt(i, 7, fTrueZ);

            b.setElementAtIndex(i, fMeasX - fTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, fTrueY);
            a.setElementAt(i, 8, fTrueX);
            a.setElementAt(i, 9, fTrueZ);

            b.setElementAtIndex(i, fMeasY - fTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, fTrueZ);
            a.setElementAt(i, 10, fTrueX);
            a.setElementAt(i, 11, fTrueY);

            b.setElementAtIndex(i, fMeasZ - fTrueZ);
            i++;
        }

        final var unknowns = Utils.solve(a, b);

        final var bx = unknowns.getElementAtIndex(0);
        final var by = unknowns.getElementAtIndex(1);
        final var bz = unknowns.getElementAtIndex(2);
        final var sx = unknowns.getElementAtIndex(3);
        final var sy = unknowns.getElementAtIndex(4);
        final var sz = unknowns.getElementAtIndex(5);
        final var mxy = unknowns.getElementAtIndex(6);
        final var mxz = unknowns.getElementAtIndex(7);
        final var myx = unknowns.getElementAtIndex(8);
        final var myz = unknowns.getElementAtIndex(9);
        final var mzx = unknowns.getElementAtIndex(10);
        final var mzy = unknowns.getElementAtIndex(11);

        fillBiases(bx, by, bz);
        fillMa(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Fills estimated biases array with estimated values.
     *
     * @param bx x coordinate of bias.
     * @param by y coordinate of bias.
     * @param bz z coordinate of bias.
     */
    private void fillBiases(final double bx, final double by, final double bz) {
        if (estimatedBiases == null) {
            estimatedBiases = new double[BodyKinematics.COMPONENTS];
        }

        estimatedBiases[0] = bx;
        estimatedBiases[1] = by;
        estimatedBiases[2] = bz;
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
}
