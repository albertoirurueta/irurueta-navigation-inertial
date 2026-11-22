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
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Calculates position, velocity, attitude and IMU biases using an INS loosely
 * coupled Kalman filter to take into account inertial measurements to
 * smooth results and taking into account an initial position.
 * Every IMU sample provided through {@link #update(BodyKinematics, double)} is corrected with the
 * currently estimated biases and integrated with the ECEF inertial navigation equations, so that the
 * frame held by this estimator is always the INS solution. A Kalman filter epoch is only run when a
 * position and velocity fix is provided through
 * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)}, which uses the INS solution
 * as the predicted state, the fix as the measurement, and then applies the closed-loop correction to
 * the INS solution. Without fixes this estimator behaves as a pure inertial navigator.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/Loosely_coupled_INS_GNSS.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/Loosely_coupled_INS_GNSS.m
 * </a>
 */
@SuppressWarnings("DuplicatedCode")
public class INSLooselyCoupledKalmanFilteredEstimator {

    /**
     * Number of units in the last place of the compared timestamps tolerated when
     * checking that the interval between Kalman filter epochs is not less than the
     * epoch interval.
     */
    private static final double EPOCH_INTERVAL_TOLERANCE_ULPS = 4.0;

    /**
     * Listener to notify events raised by this instance.
     */
    private INSLooselyCoupledKalmanFilteredEstimatorListener listener;

    /**
     * Minimum epoch interval expressed in seconds (s) between consecutive
     * Kalman filter epochs.
     * Attempting to provide a position and velocity fix when the interval since the
     * last Kalman filter epoch is less than this value, will be ignored.
     * IMU samples are always integrated regardless of this value.
     */
    private double epochInterval;

    /**
     * INS loosely coupled Kalman filter configuration parameters (usually
     * obtained through calibration).
     */
    private INSLooselyCoupledKalmanConfig config;

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
     * Contains current or initial user position, velocity and attitude.
     * This is the INS solution: it is propagated with every IMU sample and
     * corrected on every Kalman filter epoch.
     */
    private ECEFFrame frame;

    /**
     * Configuration containing uncertainty measures to set initial covariance matrix
     * within estimated state.
     * Once this estimator is initialized, covariance will be updated with new provided
     * INS measurements until convergence is reached.
     */
    private INSLooselyCoupledKalmanInitializerConfig initialConfig;

    /**
     * Current Kalman filter state containing current INS estimation along with
     * Kalman filter covariance error matrix.
     */
    private INSLooselyCoupledKalmanState state;

    /**
     * Timestamp expressed in seconds since epoch time when Kalman filter state
     * was last propagated (i.e. timestamp of the last Kalman filter epoch).
     */
    private Double lastStateTimestamp;

    /**
     * Timestamp expressed in seconds since epoch time of the last IMU sample
     * (body kinematics) that was provided to this estimator.
     */
    private Double lastKinematicsTimestamp;

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
    public INSLooselyCoupledKalmanFilteredEstimator() {
    }

    /**
     * Constructor.
     *
     * @param config INS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanConfig config) {
        this.config = new INSLooselyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final double epochInterval) {
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
    public INSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval) {
        this(epochInterval);
        this.config = new INSLooselyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param listener listener to notify events raised by this instance.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSLooselyCoupledKalmanFilteredEstimator(final Time epochInterval) {
        this(TimeConverter.convert(epochInterval.getValue().doubleValue(), epochInterval.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
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
    public INSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param frame frame containing initial user position, velocity and attitude
     *              resolved along ECEF axes.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final ECEFFrame frame) {
        this();
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens.
        }
    }

    /**
     * Constructor.
     *
     * @param config INS Kalman filter configuration parameters (usually obtained
     *               through calibration).
     * @param frame  frame containing initial user position, velocity and attitude
     *               resolved along ECEF axes.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanConfig config, final ECEFFrame frame) {
        this(frame);
        this.config = new INSLooselyCoupledKalmanConfig(config);
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final double epochInterval, final ECEFFrame frame) {
        this(epochInterval);
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param frame    frame containing initial user position, velocity and attitude
     *                 resolved along ECEF axes.
     * @param listener listener to notify events raised by this instance.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final ECEFFrame frame, final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(frame);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval, final ECEFFrame frame) {
        this(config, epochInterval);
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config   INS loosely coupled Kalman filter configuration parameters
     *                 (usually obtained through calibration).
     * @param frame    frame containing initial user position, velocity and attitude
     *                 resolved along ECEF axes.
     * @param listener listener to notify events raised by this instance.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, listener);
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setFrame(frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final Time epochInterval, final ECEFFrame frame) {
        this(epochInterval);
        try {
            setFrame(frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval, final ECEFFrame frame) {
        this(config, epochInterval);
        try {
            setFrame(frame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(final Time epochInterval, final ECEFFrame frame,
                                                    final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, listener);
        try {
            setFrame(frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, listener);
        try {
            setFrame(frame);
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
    public INSLooselyCoupledKalmanFilteredEstimator(final INSLooselyCoupledKalmanInitializerConfig initialConfig) {
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
     * @param config        INS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
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
    public INSLooselyCoupledKalmanFilteredEstimator(
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
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(initialConfig);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
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
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
    public INSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval expressed in seconds (s) between
     *                      consecutive propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
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
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
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
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param config        INS loosely coupled Kalman filter configuration
     *                      parameters (usually obtained through calibration).
     * @param epochInterval minimum epoch interval between consecutive
     *                      propagations or measurements.
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame) {
        this(frame);
        try {
            setInitialConfig(initialConfig);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param config        INS Kalman filter configuration parameters (usually obtained
     *                      through calibration).
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final ECEFFrame frame) {
        this(config, frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final ECEFFrame frame) {
        this(epochInterval, frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(frame, listener);
        try {
            setInitialConfig(initialConfig);
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame) {
        this(config, epochInterval, frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, frame, listener);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final double epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final ECEFFrame frame, final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, frame, listener);
        try {
            setInitialConfig(initialConfig);
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final double epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, frame, listener);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final ECEFFrame frame) {
        this(epochInterval, frame);
        try {
            setInitialConfig(initialConfig);
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame) {
        this(config, epochInterval, frame);
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
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final Time epochInterval, final INSLooselyCoupledKalmanInitializerConfig initialConfig,
            final ECEFFrame frame, final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(epochInterval, frame, listener);
        try {
            setInitialConfig(initialConfig);
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
     * @param initialConfig initial INS loosely coupled Kalman configuration to
     *                      set proper initial covariance during filter initialization.
     * @param frame         frame containing initial user position, velocity and attitude
     *                      resolved along ECEF axes.
     * @param listener      listener to notify events raised by this instance.
     * @throws IllegalArgumentException if provided epoch interval is negative.
     */
    public INSLooselyCoupledKalmanFilteredEstimator(
            final INSLooselyCoupledKalmanConfig config, final Time epochInterval,
            final INSLooselyCoupledKalmanInitializerConfig initialConfig, final ECEFFrame frame,
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) {
        this(config, epochInterval, frame, listener);
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
    public INSLooselyCoupledKalmanFilteredEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to notify events raised by this instance.
     *
     * @param listener listener to notify events raised by this instance.
     * @throws LockedException if this estimator is already running.
     */
    public void setListener(
            final INSLooselyCoupledKalmanFilteredEstimatorListener listener) throws LockedException {
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
     * when interval are less than this value, will be ignored.
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
     * Gets INS loosely coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param result instance where INS loosely coupled Kalman configuration
     *               parameters will be stored.
     * @return true if result instance is updated, false otherwise.
     */
    public boolean getConfig(final INSLooselyCoupledKalmanConfig result) {
        if (config != null) {
            result.copyFrom(config);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets INS loosely coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @return INS loosely coupled Kalman configuration parameters.
     */
    public INSLooselyCoupledKalmanConfig getConfig() {
        return config;
    }

    /**
     * Sets INS loosely coupled Kalman configuration parameters (usually
     * obtained through calibration).
     *
     * @param config INS loosely coupled Kalman configuration parameters
     *               to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setConfig(final INSLooselyCoupledKalmanConfig config) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.config = new INSLooselyCoupledKalmanConfig(config);
    }

    /**
     * Gets ECEF frame containing current or initial user position, velocity and
     * attitude.
     *
     * @param result instance where current ECEF frame will be stored.
     * @return true if provided result instance is updated, false otherwise.
     */
    public boolean getFrame(final ECEFFrame result) {
        if (frame != null) {
            frame.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets ECEF frame containing current or initial user position, velocity and
     * attitude.
     *
     * @return ECEF frame containing current or initial user position, velocity
     * and attitude.
     */
    public ECEFFrame getFrame() {
        return frame != null ? new ECEFFrame(frame) : null;
    }

    /**
     * Sets ECEF frame containing current or initial user position, velocity and
     * attitude.
     * Provided instance is kept (not copied) and is updated in place as IMU samples
     * are integrated and Kalman filter corrections are applied.
     * If the Kalman filter state has already been initialized, its position, velocity
     * and attitude are synchronized with provided frame, while its estimated biases and
     * covariance are preserved.
     *
     * @param frame ECEF frame containing current or initial user position, velocity
     *              and attitude to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setFrame(final ECEFFrame frame) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.frame = frame;

        if (state != null && frame != null) {
            // keep Kalman filter state synchronized with the INS solution
            state.setFrame(frame);
        }
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * INS measurements until convergence is reached.
     *
     * @param result instance where configuration data will be stored.
     * @return true if result instance was updated, false otherwise.
     */
    public boolean getInitialConfig(final INSLooselyCoupledKalmanInitializerConfig result) {
        if (initialConfig != null) {
            result.copyFrom(initialConfig);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * INS measurements until convergence is reached.
     *
     * @return initial INS loosely coupled Kalman configuration.
     */
    public INSLooselyCoupledKalmanInitializerConfig getInitialConfig() {
        return initialConfig;
    }

    /**
     * Sets initial INS loosely coupled Kalman configuration to set a proper
     * initial covariance matrix during the first Kalman filter propagation.
     * Once this estimator is initialized, covariance will be updated with new provided
     * INS measurements until convergence is reached.
     *
     * @param initialConfig initial configuration to be set.
     * @throws LockedException if this estimator is already running.
     */
    public void setInitialConfig(final INSLooselyCoupledKalmanInitializerConfig initialConfig) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.initialConfig = initialConfig;
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
     * Gets current Kalman filter state containing current INS estimation along
     * with Kalman filter covariance error matrix.
     *
     * @return current Kalman filter state containing current INS estimation
     * along with Kalman filter covariance error matrix.
     */
    public INSLooselyCoupledKalmanState getState() {
        return state != null ? new INSLooselyCoupledKalmanState(state) : null;
    }

    /**
     * Gets current Kalman filter state containing current INS estimation along
     * with Kalman filter covariance error matrix.
     * This method does not update result instance if no state is available.
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
     * Indicates whether this instance is ready to update state using available
     * IMU data (specific force and angular rates).
     *
     * @return true if ready, false otherwise.
     */
    public boolean isUpdateReady() {
        return frame != null;
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and
     * integrated with the ECEF inertial navigation equations over the interval elapsed
     * since the previous sample, regardless of the epoch interval, so that the frame
     * (and the Kalman filter state) always hold the INS-propagated solution.
     * The first provided sample only initializes the Kalman filter state and sets the
     * time reference: no navigation is performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous sample
     * are ignored.
     * This method does not run any Kalman filter epoch. Use
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)} to provide
     * a position and velocity fix.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU). If null, no navigation is performed and
     *                   only the time reference is updated.
     * @param timestamp  timestamp since epoch time when specific force and
     *                   angular rate values were updated.
     * @return true if the sample was integrated, false if it was ignored because its
     * timestamp is not greater than the timestamp of the previous sample.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if this estimator is not ready to be updated.
     * @throws INSException      if estimation fails due to numerical instabilities.
     */
    public boolean update(final BodyKinematics kinematics, final Time timestamp) throws LockedException,
            NotReadyException, INSException {
        return update(kinematics, TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Updates specific force and angular rate applied to the user's
     * body expressed in coordinates resolved along body-frame axes.
     * Every provided sample is corrected with the currently estimated biases and
     * integrated with the ECEF inertial navigation equations over the interval elapsed
     * since the previous sample, regardless of the epoch interval, so that the frame
     * (and the Kalman filter state) always hold the INS-propagated solution.
     * The first provided sample only initializes the Kalman filter state and sets the
     * time reference: no navigation is performed for it.
     * Samples whose timestamp is not greater than the timestamp of the previous sample
     * are ignored.
     * This method does not run any Kalman filter epoch. Use
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)} to provide
     * a position and velocity fix.
     *
     * @param kinematics kinematics applied to body (specific force and angular rate)
     *                   during last period of time. These measures are obtained from
     *                   an inertial unit (IMU). If null, no navigation is performed and
     *                   only the time reference is updated.
     * @param timestamp  timestamp expressed in seconds since epoch time when specific
     *                   force and angular rate values were updated.
     * @return true if the sample was integrated, false if it was ignored because its
     * timestamp is not greater than the timestamp of the previous sample.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if this estimator is not ready to be updated.
     * @throws INSException      if estimation fails due to numerical instabilities.
     */
    public boolean update(final BodyKinematics kinematics, final double timestamp) throws LockedException,
            NotReadyException, INSException {

        if (running) {
            throw new LockedException();
        }

        if (!isUpdateReady()) {
            throw new NotReadyException();
        }

        if (lastKinematicsTimestamp != null && timestamp <= lastKinematicsTimestamp) {
            return false;
        }

        try {
            running = true;

            if (listener != null) {
                listener.onUpdateStart(this);
            }

            initState();

            if (kinematics != null) {
                correctKinematics(kinematics);

                if (lastKinematicsTimestamp != null) {
                    // integrate sample over the interval elapsed since previous sample
                    ECEFInertialNavigator.navigateECEF(timestamp - lastKinematicsTimestamp, frame,
                            correctedKinematics, frame);
                }
            }

            // state always holds the INS-propagated solution
            state.setFrame(frame);

            this.kinematics = kinematics;
            lastKinematicsTimestamp = timestamp;
            if (firstKinematicsTimestamp == null) {
                firstKinematicsTimestamp = timestamp;
            }

            if (listener != null) {
                listener.onUpdateEnd(this);
            }

        } catch (final InertialNavigatorException e) {
            throw new INSException(e);
        } finally {
            running = false;
        }

        return true;
    }

    /**
     * Indicates whether this estimator is ready for state propagations (i.e. to run
     * Kalman filter epochs when position and velocity fixes are provided).
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isPropagateReady() {
        return config != null && frame != null;
    }

    /**
     * Provides a position and velocity fix (e.g. a GNSS solution) and runs a Kalman
     * filter epoch.
     * The INS-propagated solution held by this estimator is used as the predicted
     * state, provided fix is used as the measurement, the interval since the last
     * Kalman filter epoch is used as the propagation interval (on the first epoch,
     * the interval since the first IMU sample, or zero if none was provided), and the latest bias-corrected specific force is used to build the
     * transition matrix. Once the epoch is solved, the closed-loop correction is
     * applied to the INS solution (frame) and estimated biases are updated.
     * The fix is compared against the INS solution at the time of the last provided
     * IMU sample (see {@link #getLastKinematicsTimestamp()}), which is not advanced or
     * moved back to provided timestamp. For best accuracy, a fix must be provided right
     * after the IMU sample whose timestamp is closest to the timestamp of the fix:
     * otherwise the distance travelled between both timestamps is attributed to INS
     * errors.
     * Call to this method will be ignored if the interval between provided timestamp
     * and the timestamp of the last Kalman filter epoch is not positive or is less
     * than the epoch interval. A fix arriving exactly one epoch interval after the
     * previous epoch is accepted.
     *
     * @param position  ECEF position of the fix.
     * @param velocity  ECEF velocity of the fix.
     * @param timestamp timestamp expressed in seconds since epoch time when the fix
     *                  was obtained.
     * @return true if the Kalman filter epoch was run, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSException      if estimation fails due to numerical instabilities.
     */
    public boolean updatePositionAndVelocity(
            final ECEFPosition position, final ECEFVelocity velocity, final double timestamp)
            throws LockedException, NotReadyException, INSException {
        return updatePositionAndVelocity(position.getX(), position.getY(), position.getZ(),
                velocity.getVx(), velocity.getVy(), velocity.getVz(), timestamp);
    }

    /**
     * Provides a position and velocity fix (e.g. a GNSS solution) and runs a Kalman
     * filter epoch.
     * The INS-propagated solution held by this estimator is used as the predicted
     * state, provided fix is used as the measurement, the interval since the last
     * Kalman filter epoch is used as the propagation interval (on the first epoch,
     * the interval since the first IMU sample, or zero if none was provided), and the latest bias-corrected specific force is used to build the
     * transition matrix. Once the epoch is solved, the closed-loop correction is
     * applied to the INS solution (frame) and estimated biases are updated.
     * The fix is compared against the INS solution at the time of the last provided
     * IMU sample (see {@link #getLastKinematicsTimestamp()}), which is not advanced or
     * moved back to provided timestamp. For best accuracy, a fix must be provided right
     * after the IMU sample whose timestamp is closest to the timestamp of the fix:
     * otherwise the distance travelled between both timestamps is attributed to INS
     * errors.
     * Call to this method will be ignored if the interval between provided timestamp
     * and the timestamp of the last Kalman filter epoch is not positive or is less
     * than the epoch interval. A fix arriving exactly one epoch interval after the
     * previous epoch is accepted.
     *
     * @param x         ECEF x coordinate of the fix position expressed in meters (m).
     * @param y         ECEF y coordinate of the fix position expressed in meters (m).
     * @param z         ECEF z coordinate of the fix position expressed in meters (m).
     * @param vx        ECEF x coordinate of the fix velocity expressed in meters per
     *                  second (m/s).
     * @param vy        ECEF y coordinate of the fix velocity expressed in meters per
     *                  second (m/s).
     * @param vz        ECEF z coordinate of the fix velocity expressed in meters per
     *                  second (m/s).
     * @param timestamp timestamp expressed in seconds since epoch time when the fix
     *                  was obtained.
     * @return true if the Kalman filter epoch was run, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSException      if estimation fails due to numerical instabilities.
     */
    public boolean updatePositionAndVelocity(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz, final double timestamp)
            throws LockedException, NotReadyException, INSException {

        if (running) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            throw new NotReadyException();
        }

        if (lastStateTimestamp != null && isEpochRejected(timestamp, lastStateTimestamp, epochInterval)) {
            return false;
        }

        // the first epoch propagates the covariance over the time the INS solution has
        // been integrated (as Groves' reference, where the time of the last epoch is
        // initialized to the start time)
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

            INSLooselyCoupledKalmanEpochEstimator.estimate(x, y, z, vx, vy, vz, propagationInterval, state, fx, fy, fz,
                    config, state);
            lastStateTimestamp = timestamp;

            // closed-loop correction of the INS solution (state re-orthonormalizes the
            // corrected attitude when converting it into a frame)
            state.getFrame(frame);

            if (listener != null) {
                listener.onPropagateEnd(this);
            }

        } catch (final AlgebraException e) {
            throw new INSException(e);
        } finally {
            running = false;
        }

        return true;
    }

    /**
     * Provides a position and velocity fix (e.g. a GNSS solution) and runs a Kalman
     * filter epoch.
     * The INS-propagated solution held by this estimator is used as the predicted
     * state, provided fix is used as the measurement, the interval since the last
     * Kalman filter epoch is used as the propagation interval (on the first epoch,
     * the interval since the first IMU sample, or zero if none was provided), and the latest bias-corrected specific force is used to build the
     * transition matrix. Once the epoch is solved, the closed-loop correction is
     * applied to the INS solution (frame) and estimated biases are updated.
     * The fix is compared against the INS solution at the time of the last provided
     * IMU sample (see {@link #getLastKinematicsTimestamp()}), which is not advanced or
     * moved back to provided timestamp. For best accuracy, a fix must be provided right
     * after the IMU sample whose timestamp is closest to the timestamp of the fix:
     * otherwise the distance travelled between both timestamps is attributed to INS
     * errors.
     * Call to this method will be ignored if the interval between provided timestamp
     * and the timestamp of the last Kalman filter epoch is not positive or is less
     * than the epoch interval. A fix arriving exactly one epoch interval after the
     * previous epoch is accepted.
     *
     * @param position  ECEF position of the fix.
     * @param velocity  ECEF velocity of the fix.
     * @param timestamp timestamp since epoch time when the fix was obtained.
     * @return true if the Kalman filter epoch was run, false otherwise.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSException      if estimation fails due to numerical instabilities.
     */
    public boolean updatePositionAndVelocity(
            final ECEFPosition position, final ECEFVelocity velocity, final Time timestamp)
            throws LockedException, NotReadyException, INSException {
        return updatePositionAndVelocity(position, velocity, TimeConverter.convert(
                timestamp.getValue().doubleValue(), timestamp.getUnit(), TimeUnit.SECOND));
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is initialized
     * and synchronized with the current INS solution (frame).
     * This method no longer runs a Kalman filter epoch: an epoch needs a position and
     * velocity measurement, and is now triggered by
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, Time)}.
     * No listener is notified by this method.
     *
     * @param timestamp timestamp since epoch. It is ignored.
     * @return always false, since no Kalman filter measurement update is possible
     * without a position and velocity fix.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSException      never thrown. Kept for backwards compatibility.
     * @deprecated Kalman filter epochs are run by
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, Time)}. Once the
     * Kalman filter state has been initialized this method makes no action.
     */
    @Deprecated
    public boolean propagate(final Time timestamp) throws LockedException, NotReadyException, INSException {
        return propagate(TimeConverter.convert(timestamp.getValue().doubleValue(), timestamp.getUnit(),
                TimeUnit.SECOND));
    }

    /**
     * Ensures that the Kalman filter state held by this estimator is initialized
     * and synchronized with the current INS solution (frame).
     * This method no longer runs a Kalman filter epoch: an epoch needs a position and
     * velocity measurement, and is now triggered by
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)}.
     * No listener is notified by this method.
     *
     * @param timestamp timestamp expressed in seconds since epoch. It is ignored.
     * @return always false, since no Kalman filter measurement update is possible
     * without a position and velocity fix.
     * @throws LockedException   if this estimator is already running.
     * @throws NotReadyException if estimator is not ready for measurements updates.
     * @throws INSException      never thrown. Kept for backwards compatibility.
     * @deprecated Kalman filter epochs are run by
     * {@link #updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)}. Once the
     * Kalman filter state has been initialized this method makes no action.
     */
    @Deprecated
    public boolean propagate(final double timestamp) throws LockedException, NotReadyException, INSException {

        if (running) {
            throw new LockedException();
        }

        if (!isPropagateReady()) {
            throw new NotReadyException();
        }

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
        state = null;
        lastStateTimestamp = null;
        lastKinematicsTimestamp = null;
        firstKinematicsTimestamp = null;
        kinematics = null;
        correctedKinematics = null;
        frame = null;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }

    /**
     * Indicates whether a Kalman filter epoch at provided timestamp must be rejected
     * because the interval elapsed since the previous epoch is not positive or is
     * less than the epoch interval.
     * An interval equal to the epoch interval is accepted, up to the rounding errors of
     * provided timestamps.
     *
     * @param timestamp          timestamp of the epoch expressed in seconds (s).
     * @param lastStateTimestamp timestamp of the previous epoch expressed in seconds (s).
     * @param epochInterval      minimum interval between epochs expressed in seconds (s).
     * @return true if epoch must be rejected, false otherwise.
     */
    static boolean isEpochRejected(final double timestamp, final double lastStateTimestamp,
                                   final double epochInterval) {
        final var propagationInterval = timestamp - lastStateTimestamp;
        if (propagationInterval <= 0.0) {
            return true;
        }

        // the difference of two timestamps is only accurate up to their rounding errors
        // (e.g. 0.7 - 0.2 is lower than 0.5), which must not reject an epoch arriving
        // exactly one epoch interval after the previous one
        final var tolerance = EPOCH_INTERVAL_TOLERANCE_ULPS * (Math.ulp(timestamp) + Math.ulp(lastStateTimestamp));
        return propagationInterval < epochInterval - tolerance;
    }

    /**
     * Initializes initial INS loosely coupled Kalman configuration to set
     * a proper initial covariance matrix.
     * This method makes no action if an initial configuration already exists.
     */
    private void initInitialConfig() {
        if (initialConfig == null) {
            initialConfig = new INSLooselyCoupledKalmanInitializerConfig();
        }
    }

    /**
     * Initializes Kalman filter state from the current frame and the initial
     * configuration (which determines the initial covariance).
     * This method makes no action if state is already initialized.
     */
    private void initState() {
        if (state != null) {
            return;
        }

        initInitialConfig();
        final var covariance = INSLooselyCoupledKalmanInitializer.initialize(initialConfig);

        state = new INSLooselyCoupledKalmanState();
        state.setFrame(frame);
        state.setCovariance(covariance);
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
