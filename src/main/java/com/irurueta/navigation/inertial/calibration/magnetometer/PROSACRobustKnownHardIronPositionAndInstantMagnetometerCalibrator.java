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

import com.irurueta.algebra.Matrix;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.io.IOException;
import java.util.List;

/**
 * Robustly estimates magnetometer cross couplings and scaling factors
 * using PROSAC algorithm.
 * <p>
 * To use this calibrator at least 7 measurements taken at a single known
 * position and instant must be taken at 7 different unknown orientations and
 * zero velocity when common z-axis is assumed, otherwise at least 10
 * measurements are required.
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
 * Notice that this calibrator assumes that all measurements are taken in
 * a short span of time, where Earth magnetic field can be assumed to be
 * constant at provided location and instant.
 */
public class PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator extends
        RobustKnownHardIronPositionAndInstantMagnetometerCalibrator {

    /**
     * Constant defining default threshold to determine whether samples are inliers or not.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;

    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;

    /**
     * Threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on distance between estimated position and
     * distances provided for each sample.
     */
    private double threshold = DEFAULT_THRESHOLD;

    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean computeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;

    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean computeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;

    /**
     * Quality scores corresponding to each provided sample.
     * The larger the score value the better the quality of the sample.
     */
    private double[] qualityScores;

    /**
     * Constructor.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param measurements list of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
    }

    /**
     * Constructor.
     *
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final boolean commonAxisUsed) {
        super(commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final WorldMagneticModel magneticModel) {
        super(magneticModel);
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final double[] hardIron) {
        super(hardIron);
    }

    /**
     * Constructor.
     *
     * @param hardIron known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final Matrix hardIron) {
        super(hardIron);
    }

    /**
     * Constructor.
     *
     * @param hardIron  known hard-iron.
     * @param initialMm initial soft-iron matrix containing scale factors
     *                  and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final Matrix hardIron, final Matrix initialMm) {
        super(hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final NEDPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        super(position, measurements, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron) {
        super(position, measurements, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final NEDPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position position where body magnetic flux density measurements
     *                 have been taken.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(final ECEFPosition position) {
        super(position);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(position, measurements);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param listener     listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron) {
        super(position, measurements, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position        position where body magnetic flux density measurements
     *                        have been taken.
     * @param measurements    collection of body magnetic flux density
     *                        measurements with standard deviation of
     *                        magnetometer measurements taken at the same
     *                        position with zero velocity and unknown different
     *                        orientations.
     * @param initialHardIron initial hard-iron to find a solution.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix initialHardIron) {
        super(position, measurements, initialHardIron);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position     position where body magnetic flux density measurements
     *                     have been taken.
     * @param measurements collection of body magnetic flux density
     *                     measurements with standard deviation of
     *                     magnetometer measurements taken at the same
     *                     position with zero velocity and unknown different
     *                     orientations.
     * @param hardIron     known hard-iron.
     * @param initialMm    initial soft-iron matrix containing scale factors
     *                     and cross coupling errors.
     * @param listener     listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm);
    }

    /**
     * Constructor.
     *
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final ECEFPosition position, final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final boolean commonAxisUsed, final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm, listener);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param measurements  list of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final boolean commonAxisUsed) {
        super(commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param magneticModel Earth's magnetic model. If null, a default model
     *                      will be used instead.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final WorldMagneticModel magneticModel) {
        super(magneticModel);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final double[] hardIron) {
        super(hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final Matrix hardIron) {
        super(hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final Matrix hardIron, final Matrix initialMm) {
        super(hardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position) {
        super(position);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(position, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        super(position, measurements, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        super(position, measurements, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        super(position, measurements, hardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final NEDPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position) {
        super(position);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements) {
        super(position, measurements);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed) {
        super(position, measurements, commonAxisUsed);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron) {
        super(position, measurements, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron array does
     *                                  not have length 3 or if provided
     *                                  quality scores length is smaller
     *                                  than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final double[] hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron) {
        super(position, measurements, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron) {
        super(position, measurements, commonAxisUsed, hardIron);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm) {
        super(position, measurements, hardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores quality scores corresponding to each provided
     *                      measurement. The larger the score value the better
     *                      the quality of the sample.
     * @param position      position where body magnetic flux density measurements
     *                      have been taken.
     * @param measurements  collection of body magnetic flux density
     *                      measurements with standard deviation of
     *                      magnetometer measurements taken at the same
     *                      position with zero velocity and unknown different
     *                      orientations.
     * @param hardIron      known hard-iron.
     * @param initialMm     initial soft-iron matrix containing scale factors
     *                      and cross coupling errors.
     * @param listener      listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final Matrix hardIron,
            final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, hardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor.
     *
     * @param qualityScores  quality scores corresponding to each provided
     *                       measurement. The larger the score value the better
     *                       the quality of the sample.
     * @param position       position where body magnetic flux density measurements
     *                       have been taken.
     * @param measurements   collection of body magnetic flux density
     *                       measurements with standard deviation of
     *                       magnetometer measurements taken at the same
     *                       position with zero velocity and unknown different
     *                       orientations.
     * @param commonAxisUsed indicates whether z-axis is assumed to be common
     *                       for the accelerometer, gyroscope and magnetometer.
     * @param hardIron       known hard-iron.
     * @param initialMm      initial soft-iron matrix containing scale factors
     *                       and cross coupling errors.
     * @param listener       listener to handle events raised by this calibrator.
     * @throws IllegalArgumentException if provided hard-iron matrix is not
     *                                  3x1 or if soft-iron matrix is not
     *                                  3x3 or if provided quality scores
     *                                  length is smaller than 7 samples.
     */
    public PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator(
            final double[] qualityScores, final ECEFPosition position,
            final List<StandardDeviationBodyMagneticFluxDensity> measurements, final boolean commonAxisUsed,
            final Matrix hardIron, final Matrix initialMm,
            final RobustKnownHardIronPositionAndInstantMagnetometerCalibratorListener listener) {
        super(position, measurements, commonAxisUsed, hardIron, initialMm, listener);
        internalSetQualityScores(qualityScores);
    }

    /**
     * Gets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @return threshold to determine whether samples are inliers or not.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Sets threshold to determine whether samples are inliers or not when testing possible solutions.
     * The threshold refers to the amount of error on norm between measured specific forces and the
     * ones generated with estimated calibration parameters provided for each sample.
     *
     * @param threshold threshold to determine whether samples are inliers or not.
     * @throws IllegalArgumentException if provided value is equal or less than zero.
     * @throws LockedException          if calibrator is currently running.
     */
    public void setThreshold(final double threshold) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        this.threshold = threshold;
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
     * Indicates whether calibrator is ready to find a solution.
     *
     * @return true if calibrator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && qualityScores != null && qualityScores.length == measurements.size();
    }

    /**
     * Indicates whether inliers must be computed and kept.
     *
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return computeAndKeepInliers;
    }

    /**
     * Specifies whether inliers must be computed and kept.
     *
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     *                              false if inliers only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepInliersEnabled(final boolean computeAndKeepInliers) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.computeAndKeepInliers = computeAndKeepInliers;
    }

    /**
     * Indicates whether residuals must be computed and kept.
     *
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResiduals() {
        return computeAndKeepResiduals;
    }

    /**
     * Specifies whether residuals must be computed and kept.
     *
     * @param computeAndKeepResiduals true if residuals must be computed and kept,
     *                                false if residuals only need to be computed but not kept.
     * @throws LockedException if calibrator is currently running.
     */
    public void setComputeAndKeepResidualsEnabled(final boolean computeAndKeepResiduals) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.computeAndKeepResiduals = computeAndKeepResiduals;
    }

    /**
     * Estimates magnetometer calibration parameters containing soft-iron
     * scale factors and cross-coupling errors.
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

        final var innerEstimator = new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<Matrix>() {
            @Override
            public double[] getQualityScores() {
                return qualityScores;
            }

            @Override
            public double getThreshold() {
                return threshold;
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
            public void estimatePreliminarSolutions(final int[] samplesIndices, final List<Matrix> solutions) {
                computePreliminarySolutions(samplesIndices, solutions);
            }

            @Override
            public double computeResidual(final Matrix currentEstimation, final int i) {
                return computeError(measurements.get(i), currentEstimation);
            }

            @Override
            public boolean isReady() {
                return PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.this.isReady();
            }

            @Override
            public void onEstimateStart(final RobustEstimator<Matrix> estimator) {
                // no action needed
            }

            @Override
            public void onEstimateEnd(final RobustEstimator<Matrix> estimator) {
                //  no action needed
            }

            @Override
            public void onEstimateNextIteration(final RobustEstimator<Matrix> estimator, final int iteration) {
                if (listener != null) {
                    listener.onCalibrateNextIteration(
                            PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(final RobustEstimator<Matrix> estimator, final float progress) {
                if (listener != null) {
                    listener.onCalibrateProgressChange(
                            PROSACRobustKnownHardIronPositionAndInstantMagnetometerCalibrator.this, progress);
                }
            }
        });

        try {
            running = true;

            if (listener != null) {
                listener.onCalibrateStart(this);
            }

            inliersData = null;

            initialize();

            innerEstimator.setComputeAndKeepInliersEnabled(computeAndKeepInliers || refineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(computeAndKeepResiduals || refineResult);
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
        } catch (final RobustEstimatorException | IOException e) {
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
        return RobustEstimatorMethod.PROSAC;
    }

    /**
     * Indicates whether this calibrator requires quality scores for each
     * measurement or not.
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
     * @throws IllegalArgumentException if provided quality scores length
     *                                  is smaller than 4 samples.
     */
    private void internalSetQualityScores(final double[] qualityScores) {
        if (qualityScores == null || qualityScores.length < MINIMUM_MEASUREMENTS_COMMON_Z_AXIS) {
            throw new IllegalArgumentException();
        }

        this.qualityScores = qualityScores;
    }
}
