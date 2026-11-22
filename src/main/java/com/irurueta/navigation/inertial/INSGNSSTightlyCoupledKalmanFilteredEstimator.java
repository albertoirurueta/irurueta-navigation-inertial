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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSException;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Calculates position, velocity, attitude, clock offset, clock drift and IMU biases
 * using a GNSS unweighted iterated least squares estimator along with an INS tightly
 * coupled Kalman filter to take into account inertial measurements to
 * smooth results.
 * Every provided IMU sample (body kinematics) is corrected with the currently estimated
 * biases and integrated with the ECEF inertial navigation equations, so that this
 * estimator always holds the INS-propagated solution. A Kalman filter epoch is only run
 * when GNSS measurements are provided, using the INS-propagated solution as the predicted
 * state and the interval elapsed since the previous epoch as the propagation interval.
 * The resulting closed-loop correction is then applied to the INS solution.
 * The least squares GNSS estimator is only used to initialize position, velocity and
 * receiver clock until the Kalman filter state exists.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Tightly_coupled_INS_GNSS.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Tightly_coupled_INS_GNSS.m
 * </a>
 */
@SuppressWarnings("DuplicatedCode")
public class INSGNSSTightlyCoupledKalmanFilteredEstimator {

    /**
     * Internal estimator to compute least squares solution for GNSS measurements.
     */
    private final GNSSLeastSquaresPositionAndVelocityEstimator lsEstimator
            = new GNSSLeastSquaresPositionAndVelocityEstimator();

    /**
     * Listener to notify events raised by this instance.
     */
    private INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener;

    /**
     * Minimum epoch interval expressed in seconds (s) between consecutive
     * Kalman filter epochs.
     * GNSS measurements arriving when the interval since the last Kalman filter
     * epoch is less than this value, will be ignored.
     */
    private double epochInterval;

    /**
     * INS/GNSS tightly coupled Kalman filter configuration parameters (usually
     * obtained through calibration).
     */
    private INSTightlyCoupledKalmanConfig config;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> measurements;

    /**
     * Last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     */
    private BodyKinematics kinematics;

    /**
     * Contains last provided user kinematics minus currently estimated bias
     * for acceleration and angular rate values.
     */
    private BodyKinematics correctedKinematics;

    /**
     * Internally keeps user position, velocity and attitude. This is the INS
     * solution, which is integrated on every IMU sample and corrected on every
     * Kalman filter epoch.
     */
    private ECEFFrame frame;

    /**
     * Configuration containing uncertainty measures to set initial covariance matrix
     * within estimated state.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     */
    private INSTightlyCoupledKalmanInitializerConfig initialConfig;

    /**
     * Current estimation containing user ECEF position, user ECEF velocity, clock offset
     * and clock drift.
     */
    private GNSSEstimation estimation;

    /**
     * Current Kalman filter state containing current INS/GNSS estimation along with
     * Kalman filter covariance error matrix.
     */
    private INSTightlyCoupledKalmanState state;

    /**
     * Timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated.
     */
    private Double lastStateTimestamp;

    /**
     * Timestamp expressed in seconds since epoch time of the last IMU sample
     * (body kinematics) that was provided to this estimator.
     */
    private Double lastKinematicsTimestamp;

    /**
     * Indicates whether there are GNSS measurements that have not been consumed
     * yet by a Kalman filter epoch.
     */
    private boolean measurementsPending;

    /**
     * Timestamp expressed in seconds since epoch time of the first IMU sample
     * (body kinematics) that was provided to this estimator.
     * The first Kalman filter epoch propagates the covariance over the interval
     * elapsed since this timestamp.
     */
    private Double firstKinematicsTimestamp;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator() {
    }

    /**
     * Constructor.
     *
     * @param config INS/GNSS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final INSTightlyCoupledKalmanConfig config) {
        this.config = new INSTightlyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final double epochInterval) {
        try {
            setEpochInterval(epochInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval) {
        this(epochInterval);
        this.config = new INSTightlyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param config   INS/GNSS tightly coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final Time epochInterval) {
        this(TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final INSTightlyCoupledKalmanConfig config,
                                                        final Time epochInterval) {
        this(config, TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param c body-to-ECEF coordinate transformation defining the initial body
     *          attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this();
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config INS/GNSS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     * @param c      body-to-ECEF coordinate transformation defining the initial body
     *               attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        this.config = new INSTightlyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param c        body-to-ECEF coordinate transformation defining the initial body
     *                 attitude.
     * @param listener listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final CoordinateTransformation c, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config   INS/GNSS tightly coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param c        body-to-ECEF coordinate transformation defining the initial body
     *                 attitude.
     * @param listener listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final Time epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, listener);
        try {
            setCoordinateTransformation(c);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this();
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this(config);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this(epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(initialConfig);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this(config, epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this(epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig) {
        this(config, epochInterval);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(config, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final double epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSTightlyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS/GNSS tightly coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS tightly coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimator(
            final INSTightlyCoupledKalmanConfig config, final Time epochInterval,
            final INSTightlyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c,
            final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(config, epochInterval, c, listener);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Gets listener to notify events raised by this instance.
     *
     * @return listener to notify events raised by this instance.
     */
    public INSGNSSTightlyCoupledKalmanFilteredEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(final INSGNSSTightlyCoupledKalmanFilteredEstimatorListener listener)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets minimum epoch interval expressed in seconds (s) between consecutive
     * propagations or measurements expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval between consecutive propagations or
     * measurements.
     */
    public double getEpochInterval() {
        return epochInterval;
    }

    /**
     * Sets minimum epoch interval expressed in seconds (s) between consecutive
     * propagations or measurements expressed in seconds.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public void setEpochInterval(final double epochInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (epochInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        this.epochInterval = epochInterval;
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param result instance where minimum epoch interval will be stored.
     */
    public void getEpochIntervalAsTime(final Time result) {
        result.setValue(epochInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval.
     */
    public Time getEpochIntervalAsTime() {
        return new Time(epochInterval, TimeUnit.SECOND);
    }

    /**
     * Sets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param epochInterval minimum epoch interval.
     * @throws LockedException          if this estimator is already running.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public void setEpochInterval(final Time epochInterval) throws LockedException {
        final var epochIntervalSeconds = TimeConverter.convert(epochInterval.getValue().doubleValue(),
                epochInterval.getUnit(), TimeUnit.SECOND);
        setEpochInterval(epochIntervalSeconds);
    }

    /**
     * Gets INS/GNSS tightly coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param result instance where INS/GNSS tightly coupled Kalman configuration
     *               parameters will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getConfig(final INSTightlyCoupledKalmanConfig result) {
        if (config != null) {
            result.copyFrom(config);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets INS/GNSS tightly coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @return INS/GNSS tightly coupled Kalman configuration parameters.
     */
    public INSTightlyCoupledKalmanConfig getConfig() {
        return config;
    }

    /**
     * Sets INS/GNSS tightly coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param config INS/GNSS tightly coupled Kalman configuration parameters
     *               to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setConfig(final INSTightlyCoupledKalmanConfig config) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.config = new INSTightlyCoupledKalmanConfig(config);
    }

    /**
     * Gets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @return body-to-ECEF coordinate transformation.
     */
    public CoordinateTransformation getCoordinateTransformation() {
        return frame != null ? frame.getCoordinateTransformation() : null;
    }

    /**
     * Gets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @param result instance where body-to-ECEF data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getCoordinateTransformation(final CoordinateTransformation result) {
        if (frame != null) {
            frame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets body-to-ECEF coordinate transformation defining the body attitude.
     * This can be used to set the initial body attitude before starting the
     * estimator, or to update body attitude between INS/GNSS measurement updates.
     *
     * @param c body-to-ECEF coordinate transformation to be set.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid (is not a
     *                                                       body-to-ECEF transformation).
     * @throws LockedException                               if this estimator is already running.
     */
    public void setCoordinateTransformation(final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        if (running) {
            throw new LockedException();
        }

        initFrame();
        frame.setCoordinateTransformation(c);
    }

    /**
     * Gets initial INS tightly coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param result instance where configuration data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getInitialConfig(final INSTightlyCoupledKalmanInitializerConfig result) {
        if (initialConfig != null) {
            result.copyFrom(initialConfig);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets initial INS tightly coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @return initial INS tightly coupled Kalman configuration.
     */
    public INSTightlyCoupledKalmanInitializerConfig getInitialConfig() {
        return initialConfig;
    }

    /**
     * Sets initial INS tightly coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param initialConfig initial configuration to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setInitialConfig(final INSTightlyCoupledKalmanInitializerConfig initialConfig) throws LockedException {

        if (running) {
            throw new LockedException();
        }

        this.initialConfig = initialConfig;
    }

    /**
     * Gets last updated GNSS measurements of a collection of satellites.
     *
     * @return last updated GNSS measurements of a collection of satellites.
     */
    public Collection<GNSSMeasurement> getMeasurements() {
        if (measurements == null) {
            return null;
        }

        final var result = new ArrayList<GNSSMeasurement>();
        for (final var measurement : measurements) {
            result.add(new GNSSMeasurement(measurement));
        }
        return result;
    }

    /**
     * Gets last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     *
     * @return last provided user kinematics.
     */
    public BodyKinematics getKinematics() {
        return kinematics != null ? new BodyKinematics(kinematics) : null;
    }

    /**
     * Gets last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getKinematics(final BodyKinematics result) {
        if (kinematics != null) {
            result.copyFrom(kinematics);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets corrected kinematics which are the last provided user kinematics after
     * removal of the biases estimated by the Kalman filter.
     *
     * @return corrected kinematics.
     * @see #getKinematics()
     */
    public BodyKinematics getCorrectedKinematics() {
        return correctedKinematics != null ? new BodyKinematics(correctedKinematics) : null;
    }

    /**
     * Gets corrected kinematics which are the last provided user kinematics after
     * removal of the biases estimated by the Kalman filter.
     *
     * @param result instance where corrected body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getCorrectedKinematics(final BodyKinematics result) {
        if (correctedKinematics != null) {
            result.copyFrom(correctedKinematics);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     *
     * @return current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     */
    public GNSSEstimation getEstimation() {
        return estimation != null ? new GNSSEstimation(estimation) : null;
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     * This method does not update result instance if no estimation is available.
     *
     * @param result instance where estimation will be stored.
     * @return true if result estimation was updated, false otherwise.
     */
    public boolean getEstimation(final GNSSEstimation result) {
        if (estimation != null) {
            result.copyFrom(estimation);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current Kalman filter state containing current INS/GNSS estimation along
     * with Kalman filter covariance error matrix.
     *
     * @return current Kalman filter state containing current INS/GNSS estimation
     * along with Kalman filter covariance error matrix.
     */
    public INSTightlyCoupledKalmanState getState() {
        return state != null ? new INSTightlyCoupledKalmanState(state) : null;
    }

    /**
     * Gets current Kalman filter state containing current INS/GNSS estimation along
     * with Kalman filter covariance error matrix.
     * This method does not update result instance if no state is available.
     *
     * @param result instance where state will be stored.
     * @return true if result state was updated, false otherwise.
     */
    public boolean getState(final INSTightlyCoupledKalmanState result) {
        if (state != null) {
            result.copyFrom(state);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated.
     *
     * @return timestamp expressed in seconds since epoch time when Kalman filter
     * state was last propagated.
     */
    public Double getLastStateTimestamp() {
        return lastStateTimestamp;
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @param result instance where timestamp since epoch time when Kalman filter
     *               state was last propagated will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastStateTimestampAsTime(final Time result) {
        if (lastStateTimestamp != null) {
            result.setValue(lastStateTimestamp);
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @return timestamp since epoch time when Kalman filter state was last
     * propagated.
     */
    public Time getLastStateTimestampAsTime() {
        return lastStateTimestamp != null ? new Time(lastStateTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Gets timestamp expressed in seconds since epoch time of the last IMU sample
     * (body kinematics) that was provided to this estimator.
     *
     * @return timestamp expressed in seconds since epoch time of the last IMU
     * sample, or null if no sample has been provided yet.
     */
    public Double getLastKinematicsTimestamp() {
        return lastKinematicsTimestamp;
    }

    /**
     * Gets timestamp since epoch time of the last IMU sample (body kinematics) that
     * was provided to this estimator.
     *
     * @param result instance where timestamp since epoch time of the last IMU
     *               sample will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastKinematicsTimestampAsTime(final Time result) {
        if (lastKinematicsTimestamp != null) {
            result.setValue(lastKinematicsTimestamp);
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets timestamp since epoch time of the last IMU sample (body kinematics) that
     * was provided to this estimator.
     *
     * @return timestamp since epoch time of the last IMU sample, or null if no
     * sample has been provided yet.
     */
    public Time getLastKinematicsTimestampAsTime() {
        return lastKinematicsTimestamp != null ? new Time(lastKinematicsTimestamp, TimeUnit.SECOND) : null;
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Indicates whether provided measurements are ready to
     * be used for an update.
     *
     * @param measurements measurements to be checked.
     * @return true if estimator is ready, false otherwise.
     */
    public static boolean isUpdateMeasurementsReady(final Collection<GNSSMeasurement> measurements) {
        return GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(measurements);
    }

    /**
     * Updates GNSS measurements of this estimator when new satellite measurements
     * are available, and runs a Kalman filter epoch with them.
     * While the Kalman filter state does not exist, an unweighted iterated least
     * squares solution is computed to initialize position, velocity and receiver
     * clock; afterwards those are provided by the Kalman filter state.
     * Then, if this estimator is ready to propagate, a Kalman filter epoch is run
     * using the INS-propagated solution held by this estimator as the predicted
     * state, provided measurements as the measurement, the interval since the last
     * Kalman filter epoch as the propagation interval (zero on the first epoch), and
     * the latest bias-corrected specific force to build the transition matrix. Once
     * the epoch is solved, the closed-loop correction is applied to the INS solution
     * and estimated biases and receiver clock are updated.
     * Measurements are compared against the INS solution at the time of the last
     * provided IMU sample, which is not advanced or moved back to provided timestamp.
     * For best accuracy, measurements must be provided right after the IMU sample whose
     * timestamp is closest to theirs: otherwise the distance travelled between both
     * timestamps is attributed to INS errors.
     * Call to this method will be ignored if the interval between provided timestamp
     * and the timestamp of the last Kalman filter epoch is not positive or is less
     * than the epoch interval. Measurements arriving exactly one epoch interval after
     * the previous epoch are accepted.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp since epoch time when GNSS measurements were
     *                     updated.
     * @return true if measurements were updated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSGNSSException  if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(final Collection<GNSSMeasurement> measurements, final Time timestamp)
            throws LockedException, NotReadyException, INSGNSSException {
        return updateMeasurements(measurements, TimeConverter.convert(timestamp.getValue().doubleValue(),
                timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Updates GNSS measurements of this estimator when new satellite measurements
     * are available, and runs a Kalman filter epoch with them.
     * While the Kalman filter state does not exist, an unweighted iterated least
     * squares solution is computed to initialize position, velocity and receiver
     * clock; afterwards those are provided by the Kalman filter state.
     * Then, if this estimator is ready to propagate, a Kalman filter epoch is run
     * using the INS-propagated solution held by this estimator as the predicted
     * state, provided measurements as the measurement, the interval since the last
     * Kalman filter epoch as the propagation interval (zero on the first epoch), and
     * the latest bias-corrected specific force to build the transition matrix. Once
     * the epoch is solved, the closed-loop correction is applied to the INS solution
     * and estimated biases and receiver clock are updated.
     * Measurements are compared against the INS solution at the time of the last
     * provided IMU sample, which is not advanced or moved back to provided timestamp.
     * For best accuracy, measurements must be provided right after the IMU sample whose
     * timestamp is closest to theirs: otherwise the distance travelled between both
     * timestamps is attributed to INS errors.
     * Call to this method will be ignored if the interval between provided timestamp
     * and the timestamp of the last Kalman filter epoch is not positive or is less
     * than the epoch interval. Measurements arriving exactly one epoch interval after
     * the previous epoch are accepted.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp expressed in seconds since epoch time when GNSS measurements were
     *                     updated.
     * @return true if measurements were updated, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSGNSSException  if estimation fails due to numerical instabilities.
     */
    public boolean updateMeasurements(final Collection<GNSSMeasurement> measurements, final double timestamp)
            throws LockedException, NotReadyException, INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        if (!isUpdateMeasurementsReady(measurements)) {
            throw new NotReadyException();
        }

        if (isEpochRejected(timestamp)) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateGNSSMeasurementsStart(this);
            }

            this.measurements = new ArrayList<>(measurements);
            measurementsPending = true;

            if (state == null) {
                // least squares solution is only used to initialize position, velocity
                // and clock until the Kalman filter state exists
                lsEstimator.setMeasurements(this.measurements);
                lsEstimator.setPriorPositionAndVelocityFromEstimation(estimation);
                if (estimation != null) {
                    lsEstimator.estimate(estimation);
                } else {
                    estimation = lsEstimator.estimate();
                }

                initFrame();
                frame.setCoordinates(estimation.getX(), estimation.getY(), estimation.getZ());
                frame.setVelocityCoordinates(estimation.getVx(), estimation.getVy(), estimation.getVz());
            }

            if (listener != null) {
                listener.onUpdateGNSSMeasurementsEnd(this);
            }

        } catch (final GNSSException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }

        if (isPropagateReady()) {
            runEpoch(timestamp);
        }

        return true;
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and,
     * once position and velocity have been initialized from GNSS measurements, it is
     * integrated with the ECEF inertial navigation equations over the interval
     * elapsed since the previous sample, regardless of the epoch interval, so that
     * this estimator always holds the INS-propagated solution.
     * The first provided sample only sets the time reference: no navigation is
     * performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous
     * sample are ignored.
     * This method does not run any Kalman filter epoch. Epochs are run when GNSS
     * measurements are provided through
     * {@link #updateMeasurements(Collection, Time)}.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU). If null, no navigation is performed and
     *                   only the time reference is updated.
     * @param timestamp  timestamp since epoch time when specific force and
     *                   angular rate values were updated.
     * @return true if body kinematics values were updated, false if they were ignored
     * because their timestamp is not greater than the timestamp of the previous sample.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(
            final BodyKinematics kinematics, final Time timestamp) throws LockedException, INSGNSSException {
        return updateBodyKinematics(kinematics, TimeConverter.convert(timestamp.getValue().doubleValue(),
                timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and,
     * once position and velocity have been initialized from GNSS measurements, it is
     * integrated with the ECEF inertial navigation equations over the interval
     * elapsed since the previous sample, regardless of the epoch interval, so that
     * this estimator always holds the INS-propagated solution.
     * The first provided sample only sets the time reference: no navigation is
     * performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous
     * sample are ignored.
     * This method does not run any Kalman filter epoch. Epochs are run when GNSS
     * measurements are provided through
     * {@link #updateMeasurements(Collection, double)}.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU). If null, no navigation is performed and
     *                   only the time reference is updated.
     * @param timestamp  timestamp expressed in seconds since epoch time when specific
     *                   force and angular rate values were updated.
     * @return true if body kinematics values were updated, false if they were ignored
     * because their timestamp is not greater than the timestamp of the previous sample.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(final BodyKinematics kinematics, final double timestamp) throws LockedException,
            INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        if (lastKinematicsTimestamp != null && timestamp <= lastKinematicsTimestamp) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateBodyKinematicsStart(this);
            }

            initFrame();

            if (kinematics != null) {
                correctKinematics(kinematics);

                if (lastKinematicsTimestamp != null) {
                    // integrate sample over the interval elapsed since previous sample.
                    // Every sample is integrated so that attitude changes are never lost,
                    // even before GNSS measurements initialize position and velocity
                    ECEFInertialNavigator.navigateECEF(timestamp - lastKinematicsTimestamp, frame,
                            correctedKinematics, frame);

                    // estimation and state always hold the INS-propagated solution.
                    // Position and velocity are only known once GNSS measurements have
                    // been provided
                    if (estimation != null) {
                        estimation.setPositionCoordinates(frame.getX(), frame.getY(), frame.getZ());
                        estimation.setVelocityCoordinates(frame.getVx(), frame.getVy(), frame.getVz());
                    }
                    if (state != null) {
                        state.setFrame(frame);
                    }
                }
            }

            this.kinematics = kinematics;
            lastKinematicsTimestamp = timestamp;
            if (firstKinematicsTimestamp == null) {
                firstKinematicsTimestamp = timestamp;
            }

            if (listener != null) {
                listener.onUpdateBodyKinematicsEnd(this);
            }

        } catch (final InertialNavigatorException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }

        return true;
    }

    /**
     * Indicates whether this estimator is ready for state propagations (i.e. to run
     * Kalman filter epochs when GNSS measurements are provided).
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return config != null && estimation != null;
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is initialized
     * and synchronized with the current INS solution and, only if there are GNSS
     * measurements that have not been consumed yet by a Kalman filter epoch, runs
     * a Kalman filter epoch with them at provided timestamp.
     * Kalman filter epochs are normally run by
     * {@link #updateMeasurements(Collection, Time)} as soon as measurements are
     * provided, so measurements only remain pending when this estimator was not
     * ready to propagate when they were provided (e.g. because the Kalman filter
     * configuration was set afterwards). Measurements already consumed by an epoch
     * are never used again.
     * Call to this method will be ignored if this estimator is not ready to
     * propagate, or if there are pending measurements but the interval between
     * provided timestamp and the timestamp of the last Kalman filter epoch is not
     * positive or is less than the epoch interval.
     *
     * @param timestamp timestamp since epoch to propagate state.
     * @return true if a Kalman filter epoch was run, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final Time timestamp) throws LockedException, INSGNSSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is initialized
     * and synchronized with the current INS solution and, only if there are GNSS
     * measurements that have not been consumed yet by a Kalman filter epoch, runs
     * a Kalman filter epoch with them at provided timestamp.
     * Kalman filter epochs are normally run by
     * {@link #updateMeasurements(Collection, double)} as soon as measurements are
     * provided, so measurements only remain pending when this estimator was not
     * ready to propagate when they were provided (e.g. because the Kalman filter
     * configuration was set afterwards). Measurements already consumed by an epoch
     * are never used again.
     * Call to this method will be ignored if this estimator is not ready to
     * propagate, or if there are pending measurements but the interval between
     * provided timestamp and the timestamp of the last Kalman filter epoch is not
     * positive or is less than the epoch interval.
     *
     * @param timestamp timestamp expressed in seconds since epoch to propagate state.
     * @return true if a Kalman filter epoch was run, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final double timestamp) throws LockedException, INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            return false;
        }

        if (measurementsPending && !isEpochRejected(timestamp)) {
            runEpoch(timestamp);
            return true;
        }

        // measurements already consumed by an epoch are not used again: only keep
        // the state synchronized with the INS solution
        initState();
        state.setFrame(frame);
        return false;
    }

    /**
     * Resets this estimator.
     *
     * @throws LockedException if this estimator is already running.
     */
    public void reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;
        measurements = null;
        estimation = null;
        state = null;
        lastStateTimestamp = null;
        lastKinematicsTimestamp = null;
        firstKinematicsTimestamp = null;
        measurementsPending = false;
        kinematics = null;
        correctedKinematics = null;
        frame = null;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }

    /**
     * Indicates whether a Kalman filter epoch at provided timestamp must be
     * rejected because the interval elapsed since the last epoch is not positive or
     * is less than the epoch interval.
     * The first epoch is never rejected, and an epoch arriving exactly one epoch
     * interval after the previous one is accepted.
     *
     * @param timestamp timestamp expressed in seconds since epoch time.
     * @return true if epoch must be rejected, false otherwise.
     */
    private boolean isEpochRejected(final double timestamp) {
        if (lastStateTimestamp == null) {
            return false;
        }

        return INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(timestamp, lastStateTimestamp,
                epochInterval);
    }

    /**
     * Runs a Kalman filter epoch at provided timestamp using the INS-propagated
     * solution as the predicted state and the last provided GNSS measurements, and
     * applies the resulting closed-loop correction to the INS solution.
     * This method must only be called when this estimator is ready to propagate and
     * the epoch is not rejected.
     *
     * @param timestamp timestamp expressed in seconds since epoch time.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    private void runEpoch(final double timestamp) throws INSGNSSException {
        // the first epoch propagates the covariance over the time the INS solution has
        // been integrated
        final double propagationInterval;
        if (lastStateTimestamp != null) {
            propagationInterval = timestamp - lastStateTimestamp;
        } else if (firstKinematicsTimestamp != null) {
            propagationInterval = Math.max(timestamp - firstKinematicsTimestamp, 0.0);
        } else {
            propagationInterval = 0.0;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onPropagateStart(this);
            }

            // previous state is the INS-propagated solution
            initState();
            state.setFrame(frame);

            final double fx;
            final double fy;
            final double fz;
            if (correctedKinematics != null) {
                fx = correctedKinematics.getFx();
                fy = correctedKinematics.getFy();
                fz = correctedKinematics.getFz();
            } else {
                fx = 0.0;
                fy = 0.0;
                fz = 0.0;
            }

            INSTightlyCoupledKalmanEpochEstimator.estimate(measurements, propagationInterval, state, fx, fy, fz,
                    config, state);
            lastStateTimestamp = timestamp;
            measurementsPending = false;

            // closed-loop correction of the INS solution. Corrected attitude is only
            // orthonormal to first order, so it is re-orthonormalized to remain a
            // valid rotation
            state.setBodyToEcefCoordinateTransformationMatrix(orthonormalize(
                    state.getBodyToEcefCoordinateTransformationMatrix()));
            state.getGNSSEstimation(estimation);
            state.getFrame(frame);

            if (listener != null) {
                listener.onPropagateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Obtains the rotation matrix closest to provided matrix.
     * The closed-loop attitude correction of the Kalman filter multiplies the
     * attitude by a small angle approximation of a rotation, so the result slightly
     * deviates from an orthonormal matrix.
     *
     * @param m a 3x3 matrix close to a rotation matrix.
     * @return closest orthonormal matrix.
     * @throws AlgebraException if singular value decomposition fails.
     */
    private static Matrix orthonormalize(final Matrix m) throws AlgebraException {
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // closest rotation is u * v', which is the provided matrix with all its
        // singular values set to one
        final var u = decomposer.getU();
        final var v = decomposer.getV();
        v.transpose();
        u.multiply(v);
        return u;
    }

    /**
     * Initializes current ECEF frame containing user position, velocity and
     * orientation expressed an resolved in ECEF coordinates.
     * This method makes no action if an initial frame already exists.
     */
    private void initFrame() {
        if (frame == null) {
            // until GNSS measurements initialize position and velocity, the frame is
            // placed at the Earth surface so that IMU samples can be integrated
            frame = new ECEFFrame();
            frame.setCoordinates(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        }
    }

    /**
     * Initializes Kalman filter state from the current frame, the receiver clock
     * of the current estimation and the initial configuration (which determines the
     * initial covariance).
     * This method makes no action if state is already initialized.
     */
    private void initState() {
        if (state != null) {
            return;
        }

        initInitialConfig();
        final var covariance = INSTightlyCoupledKalmanInitializer.initialize(initialConfig);

        state = new INSTightlyCoupledKalmanState();
        state.setFrame(frame);
        state.setReceiverClockOffset(estimation.getClockOffset());
        state.setReceiverClockDrift(estimation.getClockDrift());
        state.setCovariance(covariance);
    }

    /**
     * Initializes initial INS tightly coupled Kalman configuration to set
     * a proper initial covariance matrix.
     * This method makes no action if an initial configuration already exists.
     */
    private void initInitialConfig() {
        if (initialConfig == null) {
            initialConfig = new INSTightlyCoupledKalmanInitializerConfig();
        }
    }

    /**
     * Corrects provided kinematics by taking into account currently estimated
     * specific force and angular rate biases.
     * This method stores the result into the variable member containing corrected
     * kinematics values.
     *
     * @param kinematics kinematics instance to be corrected.
     */
    private void correctKinematics(final BodyKinematics kinematics) {
        if (correctedKinematics == null) {
            correctedKinematics = new BodyKinematics();
        }

        final double accelBiasX;
        final double accelBiasY;
        final double accelBiasZ;
        final double gyroBiasX;
        final double gyroBiasY;
        final double gyroBiasZ;
        if (state != null) {
            accelBiasX = state.getAccelerationBiasX();
            accelBiasY = state.getAccelerationBiasY();
            accelBiasZ = state.getAccelerationBiasZ();
            gyroBiasX = state.getGyroBiasX();
            gyroBiasY = state.getGyroBiasY();
            gyroBiasZ = state.getGyroBiasZ();
        } else {
            accelBiasX = 0.0;
            accelBiasY = 0.0;
            accelBiasZ = 0.0;
            gyroBiasX = 0.0;
            gyroBiasY = 0.0;
            gyroBiasZ = 0.0;
        }

        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();
        final var angularRateX = kinematics.getAngularRateX();
        final var angularRateY = kinematics.getAngularRateY();
        final var angularRateZ = kinematics.getAngularRateZ();

        correctedKinematics.setSpecificForceCoordinates(fx - accelBiasX, fy - accelBiasY, fz - accelBiasZ);
        correctedKinematics.setAngularRateCoordinates(
                angularRateX - gyroBiasX,
                angularRateY - gyroBiasY,
                angularRateZ - gyroBiasZ);
    }
}
