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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.calibration.GyroscopeCalibrationSource;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;

import java.util.Collection;

/**
 * Estimates gyroscope biases, cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer.
 * <p>
 * This calibrator uses a linear approach to find a minimum least squared error
 * solution.
 * <p>
 * To use this calibrator at least 7 measurements at different known frames must
 * be provided. In other words, accelerometer and gyroscope (i.e. body kinematics)
 * samples must be obtained at 7 different positions, orientations and velocities
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
@SuppressWarnings("DuplicatedCode")
public class KnownFrameGyroscopeLinearLeastSquaresCalibrator implements
        KnownFrameGyroscopeCalibrator<FrameBodyKinematics, KnownFrameGyroscopeLinearLeastSquaresCalibratorListener>,
        GyroscopeCalibrationSource, UnorderedFrameBodyKinematicsGyroscopeCalibrator {

    /**
     * Indicates whether by default a common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    public static final boolean DEFAULT_USE_COMMON_Z_AXIS = false;

    /**
     * Required minimum number of measurements.
     */
    public static final int MINIMUM_MEASUREMENTS = 7;

    /**
     * Number of equations generated for each measurement.
     */
    private static final int EQUATIONS_PER_MEASUREMENT = 3;

    /**
     * Number of unknowns when common z-axis is assumed for both the accelerometer
     * and gyroscope.
     */
    private static final int COMMON_Z_AXIS_UNKNOWNS = 18;

    /**
     * Number of unknowns for the general case.
     */
    private static final int GENERAL_UNKNOWNS = 21;

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
     * When enabled, this eliminates 3 variables from Mg matrix.
     */
    private boolean commonAxisUsed = DEFAULT_USE_COMMON_Z_AXIS;

    /**
     * Listener to handle events raised by this calibrator.
     */
    private KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener;

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
     * Indicates whether calibrator is running.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
            final KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param measurements collection of body kinematics measurements taken at
     *                     different frames (positions, orientations and velocities).
     */
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
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
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements,
            final KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
        this(measurements);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(final boolean commonAxisUsed) {
        this.commonAxisUsed = commonAxisUsed;
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
            final boolean commonAxisUsed, final KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
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
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
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
    public KnownFrameGyroscopeLinearLeastSquaresCalibrator(
            final Collection<? extends FrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener) {
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
     * Indicates the type of measurement or sequence used by this calibrator.
     *
     * @return type of measurement or sequence used by this calibrator.
     */
    @Override
    public GyroscopeCalibratorMeasurementOrSequenceType getMeasurementOrSequenceType() {
        return GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT;
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
    public KnownFrameGyroscopeLinearLeastSquaresCalibratorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this calibrator.
     *
     * @param listener listener to handle events raised by this calibrator.
     * @throws LockedException if calibrator is currently running.
     */
    @Override
    public void setListener(final KnownFrameGyroscopeLinearLeastSquaresCalibratorListener listener)
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

        } catch (final AlgebraException e) {
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
     * @return column matrix containing x,y,z component of estimated gyroscope
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
     * Gets x coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return x coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasX() {
        return estimatedBiases != null ? estimatedBiases[0] : null;
    }

    /**
     * Gets y coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
     *
     * @return y coordinate of estimated gyroscope bias or null if not available.
     */
    @Override
    public Double getEstimatedBiasY() {
        return estimatedBiases != null ? estimatedBiases[1] : null;
    }

    /**
     * Gets z coordinate of estimated gyroscope bias expressed in radians per second
     * (rad/s).
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
            result.setValueCoordinatesAndUnit(
                    estimatedBiases[0], estimatedBiases[1], estimatedBiases[2], AngularSpeedUnit.RADIANS_PER_SECOND);
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
     * Internal method to perform calibration when common z-axis is assumed for both
     * the accelerometer and gyroscope.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateCommonAxis() throws AlgebraException {
        // The gyroscope model is:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue + w

        // Ideally a least squares solution tries to minimize noise component, so:
        // Ωmeas = bg + (I + Mg) * Ωtrue + Gg * ftrue

        // Hence:
        // [Ωmeasx] = [bx] + ( [1   0   0] + [sx    mxy    mxz]) [Ωtruex] + [g11   g12   g13][ftruex]
        // [Ωmeasy]   [by]     [0   1   0]   [myx   sy     myz]  [Ωtruey]   [g21   g22   g23][ftruey]
        // [Ωmeasz]   [bz]     [0   0   1]   [mzx   mzy    sz ]  [Ωtruez]   [g31   g32   g33][ftruez]

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

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myz, g11, g12, g13, g21, g22, g23, g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex = bx + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey = by + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez = bz + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [1   0   0   Ωtruex  0       0       Ωtruey  Ωtruez  0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][bx ] =  [Ωmeasx - Ωtruex]
        // [0   1   0   0       Ωtruey  0       0       0       Ωtruez  0       0       0       ftruex  ftruey  ftruez  0       0       0     ][by ]    [Ωmeasy - Ωtruey]
        // [0   0   1   0       0       Ωtruez  0       0       0       0       0       0       0       0       0       ftruex  ftruey  ftruez][bz ]    [Ωmeasz - Ωtruez]
        //                                                                                                                                              [sx ]
        //                                                                                                                                              [sy ]
        //                                                                                                                                              [sz ]
        //                                                                                                                                              [mxy]
        //                                                                                                                                              [mxz]
        //                                                                                                                                              [myz]
        //                                                                                                                                              [g11]
        //                                                                                                                                              [g12]
        //                                                                                                                                              [g13]
        //                                                                                                                                              [g21]
        //                                                                                                                                              [g22]
        //                                                                                                                                              [g23]
        //                                                                                                                                              [g31]
        //                                                                                                                                              [g32]
        //                                                                                                                                              [g33]

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

            final var omegaMeasX = measuredKinematics.getAngularRateX();
            final var omegaMeasY = measuredKinematics.getAngularRateY();
            final var omegaMeasZ = measuredKinematics.getAngularRateZ();

            final var omegaTrueX = expectedKinematics.getAngularRateX();
            final var omegaTrueY = expectedKinematics.getAngularRateY();
            final var omegaTrueZ = expectedKinematics.getAngularRateZ();

            final var fTrueX = expectedKinematics.getFx();
            final var fTrueY = expectedKinematics.getFy();
            final var fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, omegaTrueX);
            a.setElementAt(i, 6, omegaTrueY);
            a.setElementAt(i, 7, omegaTrueZ);
            a.setElementAt(i, 9, fTrueX);
            a.setElementAt(i, 10, fTrueY);
            a.setElementAt(i, 11, fTrueZ);

            b.setElementAtIndex(i, omegaMeasX - omegaTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, omegaTrueY);
            a.setElementAt(i, 8, omegaTrueZ);
            a.setElementAt(i, 12, fTrueX);
            a.setElementAt(i, 13, fTrueY);
            a.setElementAt(i, 14, fTrueZ);

            b.setElementAtIndex(i, omegaMeasY - omegaTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, omegaTrueZ);
            a.setElementAt(i, 15, fTrueX);
            a.setElementAt(i, 16, fTrueY);
            a.setElementAt(i, 17, fTrueZ);

            b.setElementAtIndex(i, omegaMeasZ - omegaTrueZ);
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
        final var g11 = unknowns.getElementAtIndex(9);
        final var g12 = unknowns.getElementAtIndex(10);
        final var g13 = unknowns.getElementAtIndex(11);
        final var g21 = unknowns.getElementAtIndex(12);
        final var g22 = unknowns.getElementAtIndex(13);
        final var g23 = unknowns.getElementAtIndex(14);
        final var g31 = unknowns.getElementAtIndex(15);
        final var g32 = unknowns.getElementAtIndex(16);
        final var g33 = unknowns.getElementAtIndex(17);

        fillBiases(bx, by, bz);
        fillMg(sx, sy, sz, mxy, mxz, 0.0, myz, 0.0, 0.0);
        fillGg(g11, g12, g13, g21, g22, g23, g31, g32, g33);
    }

    /**
     * Internal method to perform general calibration.
     *
     * @throws AlgebraException if there are numerical errors.
     */
    private void calibrateGeneral() throws AlgebraException {
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

        // Where the unknowns are: bx, by, bz, sx, sy, sz, mxy mxz, myx, myz, mzx, mzy, g11, g12, g13, g21, g22, g23,
        // g31, g32, g33
        // Reordering:
        // Ωmeasx = bx + Ωtruex + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy = by + myx * Ωtruex + Ωtruey + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz = bz + mzx * Ωtruex + mzy * Ωtruey + Ωtruez + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // Ωmeasx - Ωtruex = bx + sx * Ωtruex + mxy * Ωtruey + mxz * Ωtruez + g11 * ftruex + g12 * ftruey + g13 * ftruez
        // Ωmeasy - Ωtruey = by + myx * Ωtruex + sy * Ωtruey + myz * Ωtruez + g21 * ftruex * g22 * ftruey + g23 * ftruez
        // Ωmeasz - Ωtruez = bz + mzx * Ωtruex + mzy * Ωtruey + sz * Ωtruez + g31 * ftruex + g32 * ftruey + g33 * ftruez

        // [1   0   0   Ωtruex  0       0       Ωtruey  Ωtruez  0       0       0       0       ftruex  ftruey  ftruez  0       0       0       0       0       0     ][bx ] =  [Ωmeasx - Ωtruex]
        // [0   1   0   0       Ωtruey  0       0       0       Ωtruex  Ωtruez  0       0       0       0       0       ftruex  ftruey  ftruez  0       0       0     ][by ]    [Ωmeasy - Ωtruey]
        // [0   0   1   0       0       Ωtruez  0       0       0       0       Ωtruex  Ωtruey  0       0       0       0       0       0       ftruex  ftruey  ftruez][bz ]    [Ωmeasz - Ωtruez]
        //                                                                                                                                                             [sx ]
        //                                                                                                                                                             [sy ]
        //                                                                                                                                                             [sz ]
        //                                                                                                                                                             [mxy]
        //                                                                                                                                                             [mxz]
        //                                                                                                                                                             [myx]
        //                                                                                                                                                             [myz]
        //                                                                                                                                                             [mzx]
        //                                                                                                                                                             [mzy]
        //                                                                                                                                                             [g11]
        //                                                                                                                                                             [g12]
        //                                                                                                                                                             [g13]
        //                                                                                                                                                             [g21]
        //                                                                                                                                                             [g22]
        //                                                                                                                                                             [g23]
        //                                                                                                                                                             [g31]
        //                                                                                                                                                             [g32]
        //                                                                                                                                                             [g33]

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

            final var omegaMeasX = measuredKinematics.getAngularRateX();
            final var omegaMeasY = measuredKinematics.getAngularRateY();
            final var omegaMeasZ = measuredKinematics.getAngularRateZ();

            final var omegaTrueX = expectedKinematics.getAngularRateX();
            final var omegaTrueY = expectedKinematics.getAngularRateY();
            final var omegaTrueZ = expectedKinematics.getAngularRateZ();

            final var fTrueX = expectedKinematics.getFx();
            final var fTrueY = expectedKinematics.getFy();
            final var fTrueZ = expectedKinematics.getFz();

            a.setElementAt(i, 0, 1.0);
            a.setElementAt(i, 3, omegaTrueX);
            a.setElementAt(i, 6, omegaTrueY);
            a.setElementAt(i, 7, omegaTrueZ);
            a.setElementAt(i, 12, fTrueX);
            a.setElementAt(i, 13, fTrueY);
            a.setElementAt(i, 14, fTrueZ);

            b.setElementAtIndex(i, omegaMeasX - omegaTrueX);
            i++;

            a.setElementAt(i, 1, 1.0);
            a.setElementAt(i, 4, omegaTrueY);
            a.setElementAt(i, 8, omegaTrueX);
            a.setElementAt(i, 9, omegaTrueZ);
            a.setElementAt(i, 15, fTrueX);
            a.setElementAt(i, 16, fTrueY);
            a.setElementAt(i, 17, fTrueZ);

            b.setElementAtIndex(i, omegaMeasY - omegaTrueY);
            i++;

            a.setElementAt(i, 2, 1.0);
            a.setElementAt(i, 5, omegaTrueZ);
            a.setElementAt(i, 10, omegaTrueX);
            a.setElementAt(i, 11, omegaTrueY);
            a.setElementAt(i, 18, fTrueX);
            a.setElementAt(i, 19, fTrueY);
            a.setElementAt(i, 20, fTrueZ);

            b.setElementAtIndex(i, omegaMeasZ - omegaTrueZ);
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
        final var g11 = unknowns.getElementAtIndex(12);
        final var g12 = unknowns.getElementAtIndex(13);
        final var g13 = unknowns.getElementAtIndex(14);
        final var g21 = unknowns.getElementAtIndex(15);
        final var g22 = unknowns.getElementAtIndex(16);
        final var g23 = unknowns.getElementAtIndex(17);
        final var g31 = unknowns.getElementAtIndex(18);
        final var g32 = unknowns.getElementAtIndex(19);
        final var g33 = unknowns.getElementAtIndex(20);

        fillBiases(bx, by, bz);
        fillMg(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
        fillGg(g11, g12, g13, g21, g22, g23, g31, g32, g33);
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
    private void fillMg(final double sx, final double sy, final double sz,
                        final double mxy, final double mxz, final double myx,
                        final double myz, final double mzx, final double mzy) throws WrongSizeException {
        if (estimatedMg == null) {
            estimatedMg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
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
    }

    /**
     * Fills G-dependant cross biases.
     *
     * @param g11 element 1,1 of G-dependant cross biases matrix.
     * @param g12 element 1,2 of G-dependant cross biases matrix.
     * @param g13 element 1,3 of G-dependant cross biases matrix.
     * @param g21 element 2,1 of G-dependant cross biases matrix.
     * @param g22 element 2,2 of G-dependant cross biases matrix.
     * @param g23 element 2,3 of G-dependant cross biases matrix.
     * @param g31 element 3,1 of G-dependant cross biases matrix.
     * @param g32 element 3,2 of G-dependant cross biases matrix.
     * @param g33 element 3,3 of G-dependant cross biases matrix.
     * @throws WrongSizeException never happens.
     */
    private void fillGg(final double g11, final double g12, final double g13,
                        final double g21, final double g22, final double g23,
                        final double g31, final double g32, final double g33) throws WrongSizeException {
        if (estimatedGg == null) {
            estimatedGg = new Matrix(BodyKinematics.COMPONENTS, BodyKinematics.COMPONENTS);
        }

        estimatedGg.setElementAt(0, 0, g11);
        estimatedGg.setElementAt(0, 1, g12);
        estimatedGg.setElementAt(0, 2, g13);

        estimatedGg.setElementAt(1, 0, g21);
        estimatedGg.setElementAt(1, 1, g22);
        estimatedGg.setElementAt(1, 2, g23);

        estimatedGg.setElementAt(2, 0, g31);
        estimatedGg.setElementAt(2, 1, g32);
        estimatedGg.setElementAt(2, 2, g33);
    }
}
