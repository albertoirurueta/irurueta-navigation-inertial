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

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.navigation.gnss.GNSSException;
import com.irurueta.navigation.gnss.GNSSLeastSquaresPositionAndVelocityEstimator;
import com.irurueta.navigation.gnss.GNSSMeasurement;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;
import java.util.ArrayList;
import java.util.Collection;

/**
 * Calculates position, velocity, attitude and IMU biases using a GNSS unweighted
 * iterated least squares estimator along with an INS loosely coupled Kalman filter
 * to take into account inertial measurements to smooth results.
 * Every IMU sample provided through {@link #updateBodyKinematics(BodyKinematics, double)} is
 * integrated with the ECEF inertial navigation equations, so that this estimator always holds the
 * INS-propagated solution. Every time GNSS measurements are provided through
 * {@link #updateMeasurements(Collection, double)}, their least squares position and velocity
 * solution is used as the measurement of a Kalman filter epoch, valid at the timestamp of the
 * measurements, whose closed-loop correction is applied to the INS solution. The first GNSS
 * solution initializes the position and velocity of the INS solution.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Loosely_coupled_INS_GNSS.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Loosely_coupled_INS_GNSS.m
 * </a>
 */
@SuppressWarnings("DuplicatedCode")
public class INSGNSSLooselyCoupledKalmanFilteredEstimator {

    /**
     * Internal estimator to compute least squares solution for GNSS measurements.
     */
    private final GNSSLeastSquaresPositionAndVelocityEstimator lsEstimator
            = new GNSSLeastSquaresPositionAndVelocityEstimator();

    /**
     * Internal INS estimator to update kinematic measures and propagate
     * estimated state.
     */
    private final INSLooselyCoupledKalmanFilteredEstimator insEstimator =
            new INSLooselyCoupledKalmanFilteredEstimator(new INSLooselyCoupledKalmanFilteredEstimatorListener() {
                @Override
                public void onUpdateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                    if (listener != null) {
                        listener.onUpdateBodyKinematicsStart(
                                INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                    }
                }

                @Override
                public void onUpdateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                    if (listener != null) {
                        listener.onUpdateBodyKinematicsEnd(
                                INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                    }
                }

                @Override
                public void onPropagateStart(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                    if (listener != null) {
                        listener.onPropagateStart(INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                    }
                }

                @Override
                public void onPropagateEnd(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                    if (listener != null) {
                        listener.onPropagateEnd(INSGNSSLooselyCoupledKalmanFilteredEstimator.this);
                    }
                }

                @Override
                public void onReset(final INSLooselyCoupledKalmanFilteredEstimator estimator) {
                    // no action needed
                }
            });

    /**
     * Listener to notify events raised by this instance.
     */
    private INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener;

    /**
     * GNSS measurements of a collection of satellites.
     */
    private Collection<GNSSMeasurement> measurements;

    /**
     * Last provided user kinematics containing applied specific force
     * and angular rates resolved in body axes.
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
     * Current Kalman filter state containing current INS estimation along with
     * Kalman filter covariance error matrix.
     */
    private INSLooselyCoupledKalmanState state;

    /**
     * Current estimation containing user ECEF position, user ECEF velocity, clock offset
     * and clock drift.
     */
    private GNSSEstimation estimation;

    /**
     * Indicates whether the position and velocity of the INS solution (frame) have
     * already been initialized with a GNSS position and velocity solution.
     */
    private boolean positionAndVelocityInitialized;

    /**
     * Indicates whether this estimator is running or not.
     */
    private boolean running;

    /**
     * Constructor.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator() {
    }

    /**
     * Constructor.
     *
     * @param config INS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanConfig config) {
        try {
            insEstimator.setConfig(config);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final double epochInterval) {
        try {
            insEstimator.setEpochInterval(epochInterval);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval) {
        this(epochInterval);
        try {
            insEstimator.setConfig(config);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final Time epochInterval) {
        this(TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval) {
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final CoordinateTransformation c)
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
     * @param config INS loosely Kalman filter configuration parameters (usually obtained
     *               through calibration).
     * @param c      body-to-ECEF coordinate transformation defining the initial body
     *               attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final CoordinateTransformation c)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        try {
            insEstimator.setConfig(config);
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
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
            throws InvalidSourceAndDestinationFrameTypeException {
        this(c);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final CoordinateTransformation c) throws InvalidSourceAndDestinationFrameTypeException {
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
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param c        body-to-ECEF coordinate transformation defining the initial body
     *                 attitude.
     * @param listener listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final CoordinateTransformation c, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final CoordinateTransformation c)
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
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the
     *                      initial body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval, final CoordinateTransformation c)
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
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param config        INS loosely Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(initialConfig);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial body
     *                      attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
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
     * @param config        INS loosely Kalman filter configuration parameters (usually
     *                      obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c)
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final CoordinateTransformation c, final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
     * @param config        INS loosely coupled Kalman filter configuration parameters
     *                      (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive propagations
     *                      or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to set
     *                      proper initial covariance during filter initialization.
     * @param c             body-to-ECEF coordinate transformation defining the initial
     *                      body attitude.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException                      if provided epoch interval
     *                                                       is negative.
     * @throws InvalidSourceAndDestinationFrameTypeException if provided coordinate
     *                                                       transformation is not valid.
     */
    public INSGNSSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final CoordinateTransformation c,
            final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
    public INSGNSSLooselyCoupledKalmanFilteredEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(final INSGNSSLooselyCoupledKalmanFilteredEstimatorListener listener)
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
        return insEstimator.getEpochInterval();
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

        insEstimator.setEpochInterval(epochInterval);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @param result instance where minimum epoch interval will be stored.
     */
    public void getEpochIntervalAsTime(final Time result) {
        insEstimator.getEpochIntervalAsTime(result);
    }

    /**
     * Gets minimum epoch interval between consecutive propagations or measurements.
     * Attempting to propagate results using Kalman filter or updating measurements
     * when intervals are less than this value, will be ignored.
     *
     * @return minimum epoch interval.
     */
    public Time getEpochIntervalAsTime() {
        return insEstimator.getEpochIntervalAsTime();
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
        if (running) {
            throw new LockedException();
        }

        insEstimator.setEpochInterval(epochInterval);
    }

    /**
     * Gets INS loosely coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param result instance where INS loosely coupled Kalman configuration
     *               parameters will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getConfig(final INSLooselyCoupledKalmanConfig result) {
        return insEstimator.getConfig(result);
    }

    /**
     * Gets INS loosely coupled Kalman configuration parameters (usually obtained
     * through calibration).
     *
     * @return INS loosely coupled Kalman configuration parameters.
     */
    public INSLooselyCoupledKalmanConfig getConfig() {
        return insEstimator.getConfig();
    }

    /**
     * Sets INS loosely coupled Kalman configuration parameters (usually obtained
     * through calibration).
     *
     * @param config INS loosely coupled Kalman configuration parameters to be
     *               set.
     * @throws LockedException if this estimator is already running.
     */
    public void setConfig(final INSLooselyCoupledKalmanConfig config) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        insEstimator.setConfig(config);
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
        insEstimator.setFrame(frame);
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param result instance where configuration data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getInitialConfig(final INSLooselyCoupledKalmanInitializerConfig result) {
        return insEstimator.getInitialConfig(result);
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration te set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @return initial INS loosely coupled Kalman configuration.
     */
    public INSLooselyCoupledKalmanInitializerConfig getInitialConfig() {
        return insEstimator.getInitialConfig();
    }

    /**
     * Sets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * GNSS and INS measurements until convergence is reached.
     *
     * @param initialConfig initial configuration to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setInitialConfig(final INSLooselyCoupledKalmanInitializerConfig initialConfig) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        insEstimator.setInitialConfig(initialConfig);
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
        final var result = insEstimator.getKinematics();
        if (result != null) {
            return result;
        } else {
            if (kinematics != null) {
                return new BodyKinematics(kinematics);
            } else {
                return null;
            }
        }
    }

    /**
     * Gets last provided user kinematics containing applied specific force and
     * angular rates resolved in body axes.
     *
     * @param result instance where last provided body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getKinematics(final BodyKinematics result) {
        if (!insEstimator.getKinematics(result)) {
            if (kinematics != null) {
                result.copyFrom(kinematics);
                return true;
            } else {
                return false;
            }
        } else {
            return true;
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
        final var result = insEstimator.getCorrectedKinematics();
        if (result != null) {
            return result;
        } else {
            if (correctedKinematics != null) {
                return new BodyKinematics(correctedKinematics);
            } else {
                return null;
            }
        }
    }

    /**
     * Gets corrected kinematics which are the last provided user kinematics after
     * removal of the biases estimated by the Kalman filter.
     *
     * @param result instance where corrected body kinematics will be stored.
     * @return true if provided result instance was updated, false otherwise.
     */
    public boolean getCorrectedKinematics(final BodyKinematics result) {
        if (!insEstimator.getCorrectedKinematics(result)) {
            if (correctedKinematics != null) {
                result.copyFrom(correctedKinematics);
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    /**
     * Gets current estimation containing user ECEF position, user ECEF velocity,
     * clock offset and clock drift.
     * This is the least squares solution of the last provided GNSS measurements, which
     * is the measurement used by the Kalman filter. The solution combining GNSS and
     * inertial measurements is the one returned by {@link #getState()}.
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
     * This is the least squares solution of the last provided GNSS measurements, which
     * is the measurement used by the Kalman filter. The solution combining GNSS and
     * inertial measurements is the one returned by {@link #getState(INSLooselyCoupledKalmanState)}.
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
     * Gets current Kalman filter state containing current user position,
     * velocity, attitude and IMU biases along with Kalman filter
     * covariance error matrix.
     *
     * @return current Kalman filter state.
     */
    public INSLooselyCoupledKalmanState getState() {
        return state != null ? new INSLooselyCoupledKalmanState(state) : null;
    }

    /**
     * Gets current Kalman filter state containing current user position,
     * velocity, attitude and IMU biases along with Kalman filter
     * covariance error matrix.
     *
     * @param result instance where state will be stored.
     * @return true if result state was updated, false otherwise.
     */
    public boolean getState(final INSLooselyCoupledKalmanState result) {
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
        return insEstimator.getLastStateTimestamp();
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was las propagated.
     *
     * @param result instance where timestamp since epoch time when Kalman filter
     *               state was last propagated will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getLastStateTimestampAsTime(final Time result) {
        return insEstimator.getLastStateTimestampAsTime(result);
    }

    /**
     * Gets timestamp since epoch time when Kalman filter state was last propagated.
     *
     * @return timestamp since epoch time when Kalman filter state was last
     * propagated.
     */
    public Time getLastStateTimestampAsTime() {
        return insEstimator.getLastStateTimestampAsTime();
    }

    /**
     * Indicates whether this estimator is running or not.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running || insEstimator.isRunning();
    }

    /**
     * Indicates whether provided measurements are ready to be
     * used for an update.
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
     * The least squares position and velocity solution of provided measurements is
     * computed using the INS solution as prior. The first solution initializes the
     * position and velocity of the INS solution (any configured attitude is preserved).
     * Then, if a Kalman filter configuration is available, a Kalman filter epoch is run
     * using the INS-propagated solution as predicted state and the least squares
     * solution as a measurement valid at provided timestamp (the solution is not
     * navigated forward in time), and the closed-loop correction is applied to the INS
     * solution.
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
     * are available.
     * The least squares position and velocity solution of provided measurements is
     * computed using the INS solution as prior. The first solution initializes the
     * position and velocity of the INS solution (any configured attitude is preserved).
     * Then, if a Kalman filter configuration is available, a Kalman filter epoch is run
     * using the INS-propagated solution as predicted state and the least squares
     * solution as a measurement valid at provided timestamp (the solution is not
     * navigated forward in time), and the closed-loop correction is applied to the INS
     * solution.
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

        final var lastStateTimestamp = insEstimator.getLastStateTimestamp();
        if (lastStateTimestamp != null && INSLooselyCoupledKalmanFilteredEstimator.isEpochRejected(timestamp,
                lastStateTimestamp, insEstimator.getEpochInterval())) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateGNSSMeasurementsStart(this);
            }

            this.measurements = new ArrayList<>(measurements);

            lsEstimator.setMeasurements(this.measurements);
            estimatePositionAndVelocity();

            if (insEstimator.isPropagateReady()) {
                // GNSS solution is the measurement of the Kalman filter epoch, valid at
                // provided timestamp. The INS-propagated solution is the predicted state
                insEstimator.updatePositionAndVelocity(estimation.getX(), estimation.getY(), estimation.getZ(),
                        estimation.getVx(), estimation.getVy(), estimation.getVz(), timestamp);
            }

            updateState();

            // notified once provided measurements have been completely processed, so
            // that this estimator already holds the corrected solution
            if (listener != null) {
                listener.onUpdateGNSSMeasurementsEnd(this);
            }
        } catch (final GNSSException | INSException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }

        return true;
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and
     * integrated with the ECEF inertial navigation equations over the interval elapsed
     * since the previous sample, regardless of the epoch interval, starting from the
     * last corrected INS solution. The first provided sample only sets the time
     * reference: no navigation is performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous sample
     * are ignored.
     * This method does not run any Kalman filter epoch: epochs are run when GNSS
     * measurements are provided.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU).
     * @param timestamp  timestamp since epoch time when specific force and
     *                   angular rate values were updated.
     * @return true if body kinematics values were integrated, false if they were
     * ignored because their timestamp is not greater than the one of the previous
     * sample.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(final BodyKinematics kinematics, final Time timestamp) throws LockedException,
            INSGNSSException {
        return updateBodyKinematics(kinematics, TimeConverter.convert(timestamp.getValue().doubleValue(),
                timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and
     * integrated with the ECEF inertial navigation equations over the interval elapsed
     * since the previous sample, regardless of the epoch interval, starting from the
     * last corrected INS solution. The first provided sample only sets the time
     * reference: no navigation is performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous sample
     * are ignored.
     * This method does not run any Kalman filter epoch: epochs are run when GNSS
     * measurements are provided.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU).
     * @param timestamp  timestamp expressed in seconds since epoch time when specific
     *                   force and angular rate values were updated.
     * @return true if body kinematics values were integrated, false if they were
     * ignored because their timestamp is not greater than the one of the previous
     * sample.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean updateBodyKinematics(final BodyKinematics kinematics, final double timestamp) throws LockedException,
            INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        try {
            running = true;

            if (initFrame()) {
                insEstimator.setFrame(frame);
            }

            // every sample is integrated, starting from the last corrected INS solution
            final var result = insEstimator.update(kinematics, timestamp);
            if (result) {
                if (this.kinematics == null) {
                    this.kinematics = insEstimator.getKinematics();
                } else {
                    insEstimator.getKinematics(this.kinematics);
                }

                if (estimation != null) {
                    updateState();
                }
            }

            if (kinematics != null) {
                correctKinematics(kinematics);
            }

            return result;

        } catch (final INSException | NotReadyException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Indicates whether this estimator is ready for state propagations.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return insEstimator.isPropagateReady() && estimation != null;
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is synchronized
     * with the current INS solution.
     * This method does not run a Kalman filter epoch: an epoch needs a position and
     * velocity measurement, and is run when GNSS measurements are provided through
     * {@link #updateMeasurements(Collection, double)}. Neither the INS solution nor
     * the GNSS estimation are modified, and no listener is notified.
     *
     * @param timestamp timestamp since epoch. It is ignored.
     * @return always false, since no Kalman filter epoch is run without GNSS
     * measurements.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    public boolean propagate(final Time timestamp) throws LockedException, INSGNSSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is synchronized
     * with the current INS solution.
     * This method does not run a Kalman filter epoch: an epoch needs a position and
     * velocity measurement, and is run when GNSS measurements are provided through
     * {@link #updateMeasurements(Collection, double)}. Neither the INS solution nor
     * the GNSS estimation are modified, and no listener is notified.
     *
     * @param timestamp timestamp expressed in seconds since epoch. It is ignored.
     * @return always false, since no Kalman filter epoch is run without GNSS
     * measurements.
     * @throws LockedException  if this estimator is already running.
     * @throws INSGNSSException if estimation fails due to numerical instabilities.
     */
    @SuppressWarnings("deprecation")
    public boolean propagate(final double timestamp) throws LockedException, INSGNSSException {

        if (running) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            return false;
        }

        try {
            running = true;

            // GNSS estimation is not replaced by the INS solution: otherwise the INS
            // solution would later be fed back to the Kalman filter as a measurement
            final var result = insEstimator.propagate(timestamp);
            updateState();

            return result;

        } catch (final INSException | NotReadyException e) {
            throw new INSGNSSException(e);
        } finally {
            running = false;
        }
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
        kinematics = null;
        correctedKinematics = null;
        frame = null;
        positionAndVelocityInitialized = false;

        insEstimator.reset();

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }

    /**
     * Initializes current ECEF frame containing user position, velocity and
     * orientation expressed and resolved in ECEF coordinates.
     * This method makes no action if an initial frame already exists.
     *
     * @return true if frame was initialized, false otherwise.
     */
    private boolean initFrame() {
        if (frame == null) {
            frame = new ECEFFrame();
            frame.setCoordinates(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Initializes state.
     */
    private void initState() {
        if (state == null) {
            state = new INSLooselyCoupledKalmanState();
        }
    }

    /**
     * Copies the Kalman filter state of the internal INS estimator, which holds the
     * current INS solution, into the state exposed by this estimator.
     * State is left unavailable if the internal INS estimator has no state yet.
     */
    private void updateState() {
        initState();
        if (!insEstimator.getState(state)) {
            state = null;
        }
    }

    /**
     * Computes the least squares position and velocity solution of current GNSS
     * measurements, using the INS solution as prior.
     * On the first solution there is no INS position yet: the least squares estimator
     * starts from its own default prior, its solution initializes the position and
     * velocity of the INS solution, and then the solution is refined once using that
     * position as prior, as it is done for any later solution.
     *
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    private void estimatePositionAndVelocity() throws LockedException, NotReadyException, GNSSException {
        final var firstSolution = !positionAndVelocityInitialized;
        if (firstSolution) {
            lsEstimator.setPriorPositionAndVelocity(null);
            estimateWithCurrentPrior();
            initializePositionAndVelocity();
        }

        lsEstimator.setPriorPositionAndVelocity(new ECEFPositionAndVelocity(frame.getX(), frame.getY(), frame.getZ(),
                frame.getVx(), frame.getVy(), frame.getVz()));
        estimateWithCurrentPrior();

        if (firstSolution) {
            initializePositionAndVelocity();
        }
    }

    /**
     * Computes the least squares position and velocity solution of current GNSS
     * measurements using the prior currently set into the least squares estimator,
     * and stores it as current estimation.
     *
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws GNSSException     if estimation fails due to numerical instabilities.
     */
    private void estimateWithCurrentPrior() throws LockedException, NotReadyException, GNSSException {
        if (estimation != null) {
            lsEstimator.estimate(estimation);
        } else {
            estimation = lsEstimator.estimate();
        }
    }

    /**
     * Sets the position and velocity of the INS solution (frame) to the ones of
     * current GNSS estimation, preserving the attitude (either the configured one, the
     * one integrated so far, or the default one if frame does not exist yet).
     *
     * @throws LockedException if internal INS estimator is already running.
     */
    private void initializePositionAndVelocity() throws LockedException {
        initFrame();
        frame.setCoordinates(estimation.getX(), estimation.getY(), estimation.getZ());
        frame.setVelocityCoordinates(estimation.getVx(), estimation.getVy(), estimation.getVz());
        insEstimator.setFrame(frame);
        positionAndVelocityInitialized = true;
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
            accelBiasX = getValueOrZero(state.getAccelerationBiasX());
            accelBiasY = getValueOrZero(state.getAccelerationBiasY());
            accelBiasZ = getValueOrZero(state.getAccelerationBiasZ());
            gyroBiasX = getValueOrZero(state.getGyroBiasX());
            gyroBiasY = getValueOrZero(state.getGyroBiasY());
            gyroBiasZ = getValueOrZero(state.getGyroBiasZ());
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

    /**
     * Returns provided value if not infinity and not NaN.
     *
     * @param value value to be returned.
     * @return value or 0.0.
     */
    private double getValueOrZero(final double value) {
        if (Double.isNaN(value) || Double.isInfinite(value)) {
            return 0.0;
        } else {
            return value;
        }
    }
}
