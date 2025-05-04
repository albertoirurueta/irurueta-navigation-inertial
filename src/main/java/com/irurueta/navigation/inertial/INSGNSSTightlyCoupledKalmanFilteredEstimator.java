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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
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
     * propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
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
     * Internally keeps user position, velocity and attitude.
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
     * are available.
     * Calls to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
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
     * are available.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param measurements GNSS measurements to be updated.
     * @param timestamp    timestamp expressed in seconds since epoch time when
     *                     GNSS measurements were updated.
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

        if (lastStateTimestamp != null && timestamp - lastStateTimestamp <= epochInterval) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateGNSSMeasurementsStart(this);
            }

            this.measurements = new ArrayList<>(measurements);

            lsEstimator.setMeasurements(this.measurements);
            lsEstimator.setPriorPositionAndVelocityFromEstimation(estimation);
            if (estimation != null) {
                lsEstimator.estimate(estimation);
            } else {
                estimation = lsEstimator.estimate();
            }

            if (listener != null) {
                listener.onUpdateGNSSMeasurementsEnd(this);
            }

        } catch (final GNSSException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }

        updateBodyKinematics(kinematics, timestamp);

        return true;
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU).
     * @param timestamp  timestamp since epoch time when specific force and
     *                   angular rate values were updated.
     * @return true if body kinematics values were updated, false otherwise.
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
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU).
     * @param timestamp  timestamp expressed in seconds since epoch time when specific
     *                   force and angular rate values were updated.
     * @return true if body kinematics values were updated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(final BodyKinematics kinematics, final double timestamp) throws LockedException,
            INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        final var propagationInterval = lastStateTimestamp != null ? timestamp - lastStateTimestamp : 0.0;
        if (lastStateTimestamp != null && propagationInterval <= epochInterval) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateBodyKinematicsStart(this);
            }

            initFrame();
            if (estimation != null) {
                frame.setCoordinates(estimation.getX(), estimation.getY(), estimation.getZ());
                frame.setVelocityCoordinates(estimation.getVx(), estimation.getVy(), estimation.getVz());
            }

            if (kinematics != null) {
                correctKinematics(kinematics);
                ECEFInertialNavigator.navigateECEF(propagationInterval, frame, correctedKinematics, frame);
            }

            this.kinematics = kinematics;

            if (listener != null) {
                listener.onUpdateBodyKinematicsEnd(this);
            }

        } catch (final InertialNavigatorException e) {
            return false;
        } finally {
            running = false;
        }

        propagate(timestamp);

        return true;
    }

    /**
     * Indicates whether this estimator is ready for state propagations.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return config != null && estimation != null;
    }

    /**
     * Propagates Kalman filter state held by this estimator at provided
     * timestamp.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param timestamp timestamp since epoch to propagate state.
     * @return true if state was propagated, false otherwise.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final Time timestamp) throws LockedException, INSGNSSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Propagates Kalman filter state held by this estimator at provided
     * timestamp.
     * Call to this method will be ignored if interval between provided timestamp
     * and last timestamp when Kalman filter was updated is less than epoch interval.
     *
     * @param timestamp timestamp expressed in seconds since epoch to propagate state.
     * @return true if state was propagated, false otherwise.
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

        final var propagationInterval = lastStateTimestamp != null ? timestamp - lastStateTimestamp : 0.0;
        if (lastStateTimestamp != null && propagationInterval <= epochInterval) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onPropagateStart(this);
            }

            if (initFrame()) {
                frame.setCoordinates(estimation.getX(), estimation.getY(), estimation.getZ());
                frame.setVelocityCoordinates(estimation.getVx(), estimation.getVy(), estimation.getVz());
            }

            if (state == null) {
                // initialize state
                initInitialConfig();
                final var covariance = INSTightlyCoupledKalmanInitializer.initialize(initialConfig);

                state = new INSTightlyCoupledKalmanState();
                state.setFrame(frame);
                state.setCovariance(covariance);
            }

            if (kinematics != null) {
                correctKinematics(kinematics);
            }

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

        return true;
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
        kinematics = null;
        correctedKinematics = null;
        frame = null;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }

    /**
     * Initializes current ECEF frame containing user position, velocity and
     * orientation expressed an resolved in ECEF coordinates.
     * This method makes no action if an initial frame already exists.
     *
     * @return true if frame was initialized, false otherwise.
     */
    private boolean initFrame() {
        if (frame == null) {
            frame = new ECEFFrame();
            return true;
        } else {
            return false;
        }
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
