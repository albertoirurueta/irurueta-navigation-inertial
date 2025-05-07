/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Implements one cycle of the loosely coupled INS/GNSS
 * Kalman filter plus closed-loop correction of all inertial states.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * <a href="https://github.com/ymjdz/MATLAB-Codes/blob/master/LC_KF_Epoch.m">
 *     https://github.com/ymjdz/MATLAB-Codes/blob/master/LC_KF_Epoch.m
 * </a>
 */
public class INSLooselyCoupledKalmanEpochEstimator {

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Number of components of position + velocity.
     */
    private static final int POS_AND_VEL_COMPONENTS = 6;

    /**
     * Constructor.
     * Prevents instantiation of helper class.
     */
    private INSLooselyCoupledKalmanEpochEstimator() {
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final var fx = bodyKinematics.getFx();
        final var fy = bodyKinematics.getFy();
        final var fz = bodyKinematics.getFz();

        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics, previousLatitude,
                config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final var fx = bodyKinematics.getFx();
        final var fy = bodyKinematics.getFy();
        final var fz = bodyKinematics.getFz();

        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz, previousLatitude, config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final INSLooselyCoupledKalmanConfig config)
            throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {

        final var x = userPosition.getX();
        final var y = userPosition.getY();
        final var z = userPosition.getZ();

        final var vx = userVelocity.getVx();
        final var vy = userVelocity.getVy();
        final var vz = userVelocity.getVz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz, previousLatitude, config,
                result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        GNSS estimated ECEF user position.
     * @param userVelocity        GNSS estimated ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final var x = userPosition.getX();
        final var y = userPosition.getY();
        final var z = userPosition.getZ();

        final var vx = userVelocity.getVx();
        final var vy = userVelocity.getVy();
        final var vz = userVelocity.getVz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final var fx = bodyKinematics.getFx();
        final var fy = bodyKinematics.getFy();
        final var fz = bodyKinematics.getFz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics, previousLatitude, config,
                result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {

        final var fx = bodyKinematics.getFx();
        final var fy = bodyKinematics.getFy();
        final var fz = bodyKinematics.getFz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, double fy, final double fz, final INSLooselyCoupledKalmanConfig config)
            throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final var prevNedPosition = new NEDPosition();
        final var prevNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                previousState.getX(), previousState.getY(), previousState.getZ(),
                previousState.getVx(), previousState.getVy(), previousState.getVz(), prevNedPosition, prevNedVelocity);

        final var previousLatitude = prevNedPosition.getLatitude();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final var result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {

        final var omegaIe = Utils.skewMatrix(new double[]{0.0, 0.0, EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final var phiMatrix = Matrix.identity(
                INSLooselyCoupledKalmanState.NUM_PARAMS, INSLooselyCoupledKalmanState.NUM_PARAMS);

        final var tmp1 = omegaIe.multiplyByScalarAndReturnNew(propagationInterval);
        final var tmp2 = phiMatrix.getSubmatrix(0, 0, 2, 2);
        tmp2.subtract(tmp1);

        phiMatrix.setSubmatrix(0, 0, 2, 2, tmp2);

        final var estCbeOld = previousState.getBodyToEcefCoordinateTransformationMatrix();
        tmp1.copyFrom(estCbeOld);
        tmp1.multiplyByScalar(propagationInterval);

        phiMatrix.setSubmatrix(0, 12, 2, 14, tmp1);
        phiMatrix.setSubmatrix(3, 9, 5, 11, tmp1);

        final var measFibb = new Matrix(BodyKinematics.COMPONENTS, 1);
        measFibb.setElementAtIndex(0, fx);
        measFibb.setElementAtIndex(1, fy);
        measFibb.setElementAtIndex(2, fz);

        estCbeOld.multiply(measFibb, tmp1);

        Utils.skewMatrix(tmp1, tmp2);
        tmp2.multiplyByScalar(-propagationInterval);

        phiMatrix.setSubmatrix(3, 0, 5, 2, tmp2);

        phiMatrix.getSubmatrix(3, 3, 5, 5, tmp1);
        tmp2.copyFrom(omegaIe);
        tmp2.multiplyByScalar(2.0 * propagationInterval);
        tmp1.subtract(tmp2);
        phiMatrix.setSubmatrix(3, 3, 5, 5, tmp1);

        final var sinPrevLat = Math.sin(previousLatitude);
        final var cosPrevLat = Math.cos(previousLatitude);
        final var sinPrevLat2 = sinPrevLat * sinPrevLat;
        final var cosPrevLat2 = cosPrevLat * cosPrevLat;

        // From (2.137)
        final var geocentricRadius = EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(EARTH_ECCENTRICITY * sinPrevLat, 2.0)) * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - EARTH_ECCENTRICITY * EARTH_ECCENTRICITY, 2.0) * sinPrevLat2);

        final var prevX = previousState.getX();
        final var prevY = previousState.getY();
        final var prevZ = previousState.getZ();
        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(prevX, prevY, prevZ);

        final var previousPositionNorm = Math.sqrt(prevX * prevX + prevY * prevY + prevZ * prevZ);

        final var estRebeOld = new Matrix(com.irurueta.navigation.frames.ECEFPosition.COMPONENTS, 1);
        estRebeOld.setElementAtIndex(0, prevX);
        estRebeOld.setElementAtIndex(1, prevY);
        estRebeOld.setElementAtIndex(2, prevZ);

        final var g = gravity.asMatrix();
        g.multiplyByScalar(-2.0 * propagationInterval / geocentricRadius);

        final var estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        estRebeOldTrans.multiplyByScalar(1.0 / previousPositionNorm);

        g.multiply(estRebeOldTrans, tmp1);

        phiMatrix.setSubmatrix(3, 6, 5, 8, tmp1);

        for (var i = 0; i < com.irurueta.navigation.frames.ECEFPosition.COMPONENTS; i++) {
            phiMatrix.setElementAt(6 + i, 3 + i, propagationInterval);
        }

        // 2. Determine approximate system noise covariance matrix using (14.82)
        final var qPrimeMatrix = new Matrix(
                INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final var gyroNoisePSD = config.getGyroNoisePSD();
        final var gyroNoiseValue = gyroNoisePSD * propagationInterval;
        for (var i = 0; i < 3; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroNoiseValue);
        }

        final var accelNoisePSD = config.getAccelerometerNoisePSD();
        final var accelNoiseValue = accelNoisePSD * propagationInterval;
        for (var i = 3; i < 6; i++) {
            qPrimeMatrix.setElementAt(i, i, accelNoiseValue);
        }

        final var accelBiasPSD = config.getAccelerometerBiasPSD();
        final var accelBiasValue = accelBiasPSD * propagationInterval;
        for (var i = 9; i < 12; i++) {
            qPrimeMatrix.setElementAt(i, i, accelBiasValue);
        }

        final var gyroBiasPSD = config.getGyroBiasPSD();
        final var gyroBiasValue = gyroBiasPSD * propagationInterval;
        for (var i = 12; i < 15; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroBiasValue);
        }

        // 3. Propagate state estimates using (3.14) noting that all states are zero
        // due to closed-loop correction.
        // x_est_propagated(1:15, 1) = 0

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final var pMatrixOld = previousState.getCovariance();

        qPrimeMatrix.multiplyByScalar(0.5);

        final var tmp3 = pMatrixOld.addAndReturnNew(qPrimeMatrix);
        final var pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp3);

        phiMatrix.transpose();
        pMatrixPropagated.multiply(phiMatrix);

        pMatrixPropagated.add(qPrimeMatrix);

        // MEASUREMENT UPDATE PHASE

        // 5. Set-up measurement matrix using (14.115)
        final var h = new Matrix(POS_AND_VEL_COMPONENTS, INSLooselyCoupledKalmanState.NUM_PARAMS);
        for (var i = 0; i < 3; i++) {
            h.setElementAt(i, POS_AND_VEL_COMPONENTS + i, -1.0);
            final var j = ECEFPosition.COMPONENTS + i;
            h.setElementAt(j, j, -1.0);
        }

        // 6. Set-up measurement noise covariance matrix assuming all components of
        // GNSS position and velocity are independent and have equal variance.
        final var r = new Matrix(POS_AND_VEL_COMPONENTS, POS_AND_VEL_COMPONENTS);

        final var posMeasSD = config.getPositionNoiseSD();
        final var posMeasSD2 = posMeasSD * posMeasSD;
        for (var i = 0; i < com.irurueta.navigation.frames.ECEFPosition.COMPONENTS; i++) {
            r.setElementAt(i, i, posMeasSD2);
        }

        final var velMeasSD = config.getVelocityNoiseSD();
        final var velMeasSD2 = velMeasSD * velMeasSD;
        for (var i = com.irurueta.navigation.frames.ECEFPosition.COMPONENTS; i < POS_AND_VEL_COMPONENTS; i++) {
            r.setElementAt(i, i, velMeasSD2);
        }

        // 7. Calculate Kalman gain using (3.21)
        final var hTrans = h.transposeAndReturnNew();

        final var tmp4 = h.multiplyAndReturnNew(pMatrixPropagated);
        tmp4.multiply(hTrans);
        tmp4.add(r);

        final var tmp5 = Utils.inverse(tmp4);

        final var k = pMatrixPropagated.multiplyAndReturnNew(hTrans);
        k.multiply(tmp5);

        // 8. Formulate measurement innovations using (14.102), noting that zero
        // lever arm is assumed here
        final var prevVx = previousState.getVx();
        final var prevVy = previousState.getVy();
        final var prevVz = previousState.getVz();

        final var deltaZ = new Matrix(POS_AND_VEL_COMPONENTS, 1);
        deltaZ.setElementAtIndex(0, x - prevX);
        deltaZ.setElementAtIndex(1, y - prevY);
        deltaZ.setElementAtIndex(2, z - prevZ);
        deltaZ.setElementAtIndex(3, vx - prevVx);
        deltaZ.setElementAtIndex(4, vy - prevVy);
        deltaZ.setElementAtIndex(5, vz - prevVz);

        // 9. Update state estimates using (3.24)
        // x_est_new = x_est_propagated + K_matrix * delta_z
        final var xEstNew = k.multiplyAndReturnNew(deltaZ);

        // 10. Update state estimation error covariance matrix using (3.25)
        k.multiply(h);
        final var pNew = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);
        pNew.subtract(k);
        pNew.multiply(pMatrixPropagated);

        // CLOSED-LOOP CORRECTION

        // Correct attitude, velocity, and position using (14.7-9)
        final var tmp6 = xEstNew.getSubmatrix(0, 0, 2, 0);
        final var tmp7 = Utils.skewMatrix(tmp6);

        final var estCbeNew = Matrix.identity(ECEFPosition.COMPONENTS, ECEFPosition.COMPONENTS);
        estCbeNew.subtract(tmp7);
        estCbeNew.multiply(estCbeOld);

        final var newVx = prevVx - xEstNew.getElementAtIndex(3);
        final var newVy = prevVy - xEstNew.getElementAtIndex(4);
        final var newVz = prevVz - xEstNew.getElementAtIndex(5);

        final var newX = prevX - xEstNew.getElementAtIndex(6);
        final var newY = prevY - xEstNew.getElementAtIndex(7);
        final var newZ = prevZ - xEstNew.getElementAtIndex(8);

        // Update IMU bias estimates
        final var newAccelerationBiasX = previousState.getAccelerationBiasX() + xEstNew.getElementAtIndex(9);
        final var newAccelerationBiasY = previousState.getAccelerationBiasY() + xEstNew.getElementAtIndex(10);
        final var newAccelerationBiasZ = previousState.getAccelerationBiasZ() + xEstNew.getElementAtIndex(11);

        final var newGyroBiasX = previousState.getGyroBiasX() + xEstNew.getElementAtIndex(12);
        final var newGyroBiasY = previousState.getGyroBiasY() + xEstNew.getElementAtIndex(13);
        final var newGyroBiasZ = previousState.getGyroBiasZ() + xEstNew.getElementAtIndex(14);

        // set result values
        result.setBodyToEcefCoordinateTransformationMatrix(estCbeNew);
        result.setVelocityCoordinates(newVx, newVy, newVz);
        result.setPositionCoordinates(newX, newY, newZ);
        result.setAccelerationBiasCoordinates(newAccelerationBiasX, newAccelerationBiasY, newAccelerationBiasZ);
        result.setGyroBiasCoordinates(newGyroBiasX, newGyroBiasY, newGyroBiasZ);
        result.setCovariance(pNew);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, bodyKinematics,
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, bodyKinematics, config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, bodyKinematics,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, bodyKinematics,
                previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, fx, fy, fz,
                config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, fx, fy, fz, config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, fx, fy, fz,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval), previousState, fx, fy, fz,
                previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, bodyKinematics,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, bodyKinematics, previousLatitude,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final INSLooselyCoupledKalmanConfig config)
            throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, fx, fy, fz, config);
    }

    /**
     * Estimated the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, fx, fy, fz,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState, fx, fy, fz, previousLatitude,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEf user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval,
                previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(), userPosition.getInhomZ(),
                userVelocity.getVx(), userVelocity.getVy(), userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final double previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(), positionAndVelocity.getZ(),
                positionAndVelocity.getVx(), positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        GNSS estimated ECEF user position.
     * @param userVelocity        GNSS estimated ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final double propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, convertAngle(previousLatitude),
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z, final double vx, final double vy, final double vz,
            final Time propagationInterval, final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz, final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config, final INSLooselyCoupledKalmanState result)
            throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState, fx, fy, fz, convertAngle(previousLatitude),
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition, final ECEFVelocity userVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState, fx, fy, fz, convertAngle(previousLatitude),
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final BodyKinematics bodyKinematics,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState, bodyKinematics,
                convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState, fx, fy, fz,
                convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity, final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState, final double fx, final double fy, final double fz,
            final Angle previousLatitude, final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState, fx, fy, fz, convertAngle(previousLatitude),
                config, result);
    }

    /**
     * Converts time instance into a value expressed in seconds.
     *
     * @param time time instance to be converted.
     * @return time value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Converts angle instance into a value expressed in radians.
     *
     * @param angle angle instance to be converted.
     * @return angle value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(), angle.getUnit(), AngleUnit.RADIANS);
    }
}
