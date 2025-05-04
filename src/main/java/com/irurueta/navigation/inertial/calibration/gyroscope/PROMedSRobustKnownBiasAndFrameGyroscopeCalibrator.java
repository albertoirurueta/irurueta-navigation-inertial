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

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.units.AngularSpeed;

import java.util.List;

/**
 * Robustly estimates gyroscope cross couplings and scaling factors
 * along with G-dependent cross biases introduced on the gyroscope by the
 * specific forces sensed by the accelerometer using a PROMedS algorithm to discard
 * outliers.
 * This estimator assumes that biases are known.
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
public class PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator extends RobustKnownBiasAndFrameGyroscopeCalibrator {

    /**
     * Default value to be used for stop threshold. Stop threshold can be used to
     * avoid keeping the algorithm unnecessarily iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once a
     * solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-5;

    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best
     * estimated threshold using median of residuals is not small enough. Once
     * a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double stopThreshold = DEFAULT_STOP_THRESHOLD;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasY known y coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     * @param biasZ known z coordinate of gyroscope bias expressed in radians per
     *              second (rad/s).
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasY    known y coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param biasZ    known z coordinate of gyroscope bias expressed in radians per
     *                 second (rad/s).
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasY        known y coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param biasZ        known z coordinate of gyroscope bias expressed in radians per
     *                     second (rad/s).
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX known x coordinate of gyroscope bias.
     * @param biasY known y coordinate of gyroscope bias.
     * @param biasZ known z coordinate of gyroscope bias.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        super(biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param biasX    known x coordinate of gyroscope bias.
     * @param biasY    known y coordinate of gyroscope bias.
     * @param biasZ    known z coordinate of gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        super(measurements, biasX, biasY, biasZ);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param biasX        known x coordinate of gyroscope bias.
     * @param biasY        known y coordinate of gyroscope bias.
     * @param biasZ        known z coordinate of gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final double[] bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final double[] bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final double[] bias,
            final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param bias known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final Matrix bias) {
        super(bias);
    }

    /**
     * Constructor.
     *
     * @param bias     known gyroscope bias.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias) {
        super(measurements, bias);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body kinematics measurements with standard
     *                     deviations taken at different frames (positions, orientations
     *                     and velocities).
     * @param bias         known gyroscope bias.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, listener);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final Matrix bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements, final Matrix bias,
            final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements) {
        super(measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed) {
        super(measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ) {
        super(biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ) {
        super(measurements, biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasY         known y coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param biasZ         known z coordinate of gyroscope bias expressed in radians per
     *                      second (rad/s).
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double biasX, final double biasY, final double biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasY          known y coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param biasZ          known z coordinate of gyroscope bias expressed in radians per
     *                       second (rad/s).
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double biasX, final double biasY, final double biasZ, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        super(biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ) {
        super(measurements, biasX, biasY, biasZ);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param biasX         known x coordinate of gyroscope bias.
     * @param biasY         known y coordinate of gyroscope bias.
     * @param biasZ         known z coordinate of gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        super(biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param biasX          known x coordinate of gyroscope bias.
     * @param biasY          known y coordinate of gyroscope bias.
     * @param biasZ          known z coordinate of gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final AngularSpeed biasX, final AngularSpeed biasY, final AngularSpeed biasZ,
            final boolean commonAxisUsed, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, biasX, biasY, biasZ, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @throws IllegalArgumentException if provided bias array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final double[] qualityScores, final double[] bias) {
        super(bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double[] bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias) {
        super(measurements, bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double[] bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided array does not have length 3 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final double[] bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(final double[] qualityScores, final Matrix bias) {
        super(bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param bias          known gyroscope bias.
     * @param listener      listener to be notified of events such as when estimation
     *                      starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final Matrix bias,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias) {
        super(measurements, bias);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body kinematics measurements with standard
     *                      deviations taken at different frames (positions, orientations
     *                      and velocities).
     * @param bias          known gyroscope bias.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final Matrix bias, final boolean commonAxisUsed) {
        super(bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed) {
        super(measurements, bias, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param measurements   list of body kinematics measurements with standard
     *                       deviations taken at different frames (positions, orientations
     *                       and velocities).
     * @param bias           known gyroscope bias.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common for
     *                       accelerometer and gyroscope.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided matrix is not 3x1 or
     *                                  if provided quality scores length is smaller
     *                                  than 6 samples.
     */
    public PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator(
            final double[] qualityScores, final List<StandardDeviationFrameBodyKinematics> measurements,
            final Matrix bias, final boolean commonAxisUsed,
            final RobustKnownBiasAndFrameGyroscopeCalibratorListener listener) {
        super(measurements, bias, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return stopThreshold;
    }

    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value,
     * the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm to iterate
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     *
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     *                      when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setStopThreshold(final double stopThreshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        this.stopThreshold = stopThreshold;
    }

    /**
     * Returns quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @return quality scores corresponding to each sample.
     */
    @Override
    public double[] getQualityScores() {
        return qualityScores;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     *
     * @param qualityScores quality scores corresponding to each sample.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than minimum required samples.
     * @throws LockedException          if calibrator is currently running.
     */
    @Override
    public void setQualityScores(final double[] qualityScores) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }

    /**
     * Indicates whether solver is ready to find a solution.
     *
     * @return true if solver is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == measurements.size();
    }

    /**
     * Estimates gyroscope calibration parameters containing bias, scale factors
     * cross-coupling errors and g-dependant cross biases.
     *
     * @throws LockedException      if calibrator is currently running.
     * @throws NotReadyException    if calibrator is not ready.
     * @throws CalibrationException if estimation fails for numerical reasons.
     */
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void calibrate() throws LockedException, NotReadyException, CalibrationException {
        if (running) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var innerEstimator = new PROMedSRobustEstimator<>(
                new PROMedSRobustEstimatorListener<PreliminaryResult>() {
                    @Override
                    public double[] getQualityScores() {
                        return qualityScores;
                    }

                    @Override
                    public double getThreshold() {
                        return stopThreshold;
                    }

                    @Override
                    public int getTotalSamples() {
                        return measurements.size();
                    }

                    @Override
                    public int getSubsetSize() {
                        return preliminarySubsetSize;
                    }

                    @Override
                    public void estimatePreliminarSolutions(
                            final int[] samplesIndices, final List<PreliminaryResult> solutions) {
                        computePreliminarySolutions(samplesIndices, solutions);
                    }

                    @Override
                    public double computeResidual(final PreliminaryResult currentEstimation, final int i) {
                        return computeError(measurements.get(i), currentEstimation);
                    }

                    @Override
                    public boolean isReady() {
                        return PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.this.isReady();
                    }

                    @Override
                    public void onEstimateStart(final RobustEstimator<PreliminaryResult> estimator) {
                        // no action needed
                    }

                    @Override
                    public void onEstimateEnd(final RobustEstimator<PreliminaryResult> estimator) {
                        // no action needed
                    }

                    @Override
                    public void onEstimateNextIteration(
                            final RobustEstimator<PreliminaryResult> estimator, final int iteration) {
                        if (listener != null) {
                            listener.onCalibrateNextIteration(
                                    PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.this, iteration);
                        }
                    }

                    @Override
                    public void onEstimateProgressChange(
                            final RobustEstimator<PreliminaryResult> estimator, final float progress) {
                        if (listener != null) {
                            listener.onCalibrateProgressChange(
                                    PROMedSRobustKnownBiasAndFrameGyroscopeCalibrator.this, progress);
                        }
                    }
                });

        try {
            running = true;

            if (listener != null) {
                listener.onCalibrateStart(this);
            }

            inliersData = null;
            innerEstimator.setUseInlierThresholds(true);
            innerEstimator.setConfidence(confidence);
            innerEstimator.setMaxIterations(maxIterations);
            innerEstimator.setProgressDelta(progressDelta);
            final var preliminaryResult = innerEstimator.estimate();
            inliersData = innerEstimator.getInliersData();

            attemptRefine(preliminaryResult);

            if (listener != null) {
                listener.onCalibrateEnd(this);
            }

        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } catch (final RobustEstimatorException e) {
            throw new CalibrationException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     *
     * @return method being used for robust estimation.
     */
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROMEDS;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement/sequence or not.
     *
     * @return true if quality scores are required, false otherwise.
     */
    @Override
    public boolean isQualityScoresRequired() {
        return true;
    }

    /**
     * Sets quality scores corresponding to each provided sample.
     * This method is used internally and does not check whether instance is
     * locked or not.
     *
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     *                                  smaller than 6 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null || qualityScores.length < MINIMUM_MEASUREMENTS) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
