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
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.units.*;

import java.io.Serial;
import java.io.Serializable;
import java.util.Objects;

/**
 * Kalman filter state for loosely coupled INS/GNSS extended kalman filter.
 */
public class INSLooselyCoupledKalmanState implements Serializable, Cloneable {

    /**
     * Number of parameters of the Kalman filter.
     */
    public static final int NUM_PARAMS = 15;

    /**
     * Serialization version. This is used to ensure compatibility of deserialization of permanently stored serialized
     * instances.
     */
    @Serial
    private static final long serialVersionUID = 0L;

    /**
     * Estimated body to ECEF coordinate transformation matrix.
     */
    private Matrix bodyToEcefCoordinateTransformationMatrix;

    /**
     * Estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    private double vx;

    /**
     * Estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     */
    private double vy;

    /**
     * Estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    private double vz;

    /**
     * X coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double x;

    /**
     * Y coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double y;

    /**
     * Z coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double z;

    /**
     * Estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double accelerationBiasX;

    /**
     * Estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double accelerationBiasY;

    /**
     * Estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double accelerationBiasZ;

    /**
     * Estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     */
    private double gyroBiasX;

    /**
     * Estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     */
    private double gyroBiasY;

    /**
     * Estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     */
    private double gyroBiasZ;

    /**
     * Estimated Kalman filter error covariance matrix.
     * Notice that covariance is expressed in terms of ECEF coordinates.
     * If accuracy of position, attitude or velocity needs to be expressed in terms
     * of NED coordinates, their respective sub-matrices of this covariance matrix
     * must be rotated, taking into account the Jacobian of the matrix transformation
     * relating both coordinates, the covariance can be expressed following the law
     * of propagation of uncertainties
     * <a href="https://en.wikipedia.org/wiki/Propagation_of_uncertainty">(https://en.wikipedia.org/wiki/Propagation_of_uncertainty)</a>
     * as: cov(f(x)) = J*cov(x)*J'.
     */
    private Matrix covariance;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanState() {
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis and
     *                                                 expressed in meters per second (m/s).
     * @param vy                                       estimated ECEF user velocity resolved around y axis and
     *                                                 expressed in meters per second (m/s).
     * @param vz                                       estimated ECEF user velocity resolved around z axis and
     *                                                 expressed in meters per second (m/s).
     * @param x                                        x coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param y                                        y coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param z                                        z coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final double vx, final double vy, final double vz,
            final double x, final double y, final double z,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param vx                estimated ECEF user velocity resolved around x axis.
     * @param vy                estimated ECEF user velocity resolved around y axis.
     * @param vz                estimated ECEF user velocity resolved around z axis.
     * @param x                 x coordinate of estimated ECEF user position.
     * @param y                 y coordinate of estimated ECEF user position.
     * @param z                 z coordinate of estimated ECEF user position.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis and
     *                          expressed in radians per second (rad/s).
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param vx                estimated ECEF user velocity resolved around x axis.
     * @param vy                estimated ECEF user velocity resolved around y axis.
     * @param vz                estimated ECEF user velocity resolved around z axis.
     * @param position          estimated ECEF user position.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis and
     *                          expressed in radians per second (rad/s).
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final Speed vx, final Speed vy, final Speed vz, final Point3D position,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param velocity          estimated ECEF user velocity.
     * @param position          estimated ECEF user position.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis and
     *                          expressed in radians per second (rad/s).
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final ECEFVelocity velocity, final ECEFPosition position,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setC(c);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param positionAndVelocity estimated ECEF user velocity and position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final ECEFPositionAndVelocity positionAndVelocity,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setC(c);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param frame             estimated user ECEF frame.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis and
     *                          expressed in radians per second (rad/s).
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis and
     *                          expressed in radians per second (rad/s).
     * @param covariance        estimated Kalman filter error covariance .
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final ECEFFrame frame,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setFrame(frame);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param vx                estimated ECEF user velocity resolved around x axis.
     * @param vy                estimated ECEF user velocity resolved around y axis.
     * @param vz                estimated ECEF user velocity resolved around z axis.
     * @param x                 x coordinate of estimated ECEF user position.
     * @param y                 y coordinate of estimated ECEF user position.
     * @param z                 z coordinate of estimated ECEF user position.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis.
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z, final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY, final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY, final AngularSpeed gyroBiasZ,
            final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param vx                estimated ECEF user velocity resolved around x axis.
     * @param vy                estimated ECEF user velocity resolved around y axis.
     * @param vz                estimated ECEF user velocity resolved around z axis.
     * @param position          estimated ECEF user position expressed in meters (m).
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis.
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final Speed vx, final Speed vy, final Speed vz, final Point3D position,
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                 body to ECEF coordinate transformation.
     * @param velocity          estimated ECEF user velocity.
     * @param position          estimated ECEF user position.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis.
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final ECEFVelocity velocity, final ECEFPosition position,
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setC(c);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param positionAndVelocity estimated ECEF user position and velocity.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final CoordinateTransformation c, final ECEFPositionAndVelocity positionAndVelocity,
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setC(c);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param frame             estimated user ECEF frame.
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX         estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY         estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ         estimated gyroscope bias resolved around z axis.
     * @param covariance        estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final ECEFFrame frame, final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setFrame(frame);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param x                                        x coordinate of estimated ECEF user position.
     * @param y                                        y coordinate of estimated ECEF user position.
     * @param z                                        z coordinate of estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final Speed vx, final Speed vy, final Speed vz,
            final Point3D position,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param velocity                                 estimated ECEF user velocity.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final ECEFVelocity velocity,
            final ECEFPosition position,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param positionAndVelocity                      estimated ECEF user position and velocity.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final ECEFPositionAndVelocity positionAndVelocity,
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ,
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param x                                        x coordinate of estimated ECEF user position.
     * @param y                                        y coordinate of estimated ECEF user position.
     * @param z                                        z coordinate of estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z, final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY, final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY, final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param position                                 estimated ECEF user position expressed in meters (m).
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final Speed vx, final Speed vy, final Speed vz,
            final Point3D position, final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param velocity                                 estimated ECEF user velocity.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final ECEFVelocity velocity,
            final ECEFPosition position, final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param positionAndVelocity                      estimated ECEF user position and velocity.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 15x15.
     */
    public INSLooselyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix, final ECEFPositionAndVelocity positionAndVelocity,
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ, final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ, final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setCovariance(covariance);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public INSLooselyCoupledKalmanState(final INSLooselyCoupledKalmanState input) {
        copyFrom(input);
    }

    /**
     * Gets estimated body to ECEF coordinate transformation matrix.
     *
     * @return estimated body to ECEF coordinate transformation matrix.
     */
    public Matrix getBodyToEcefCoordinateTransformationMatrix() {
        return bodyToEcefCoordinateTransformationMatrix;
    }

    /**
     * Sets estimated body to ECEF coordinate transformation matrix.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate
     *                                                 transformation matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setBodyToEcefCoordinateTransformationMatrix(final Matrix bodyToEcefCoordinateTransformationMatrix) {
        if (bodyToEcefCoordinateTransformationMatrix.getRows() != CoordinateTransformation.ROWS
                || bodyToEcefCoordinateTransformationMatrix.getColumns() != CoordinateTransformation.COLS) {
            throw new IllegalArgumentException();
        }
        this.bodyToEcefCoordinateTransformationMatrix = bodyToEcefCoordinateTransformationMatrix;
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    public double getVx() {
        return vx;
    }

    /**
     * Sets estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     *
     * @param vx estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    public void setVx(final double vx) {
        this.vx = vx;
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     */
    public double getVy() {
        return vy;
    }

    /**
     * Sets estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     *
     * @param vy estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     */
    public void setVy(final double vy) {
        this.vy = vy;
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public double getVz() {
        return vz;
    }

    /**
     * Sets estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     *
     * @param vz estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public void setVz(final double vz) {
        this.vz = vz;
    }

    /**
     * Sets estimated ECEF user velocity coordinates.
     *
     * @param vx estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     * @param vy estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     * @param vz estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public void setVelocityCoordinates(final double vx, final double vy, final double vz) {
        this.vx = vx;
        this.vy = vy;
        this.vz = vz;
    }

    /**
     * Gets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getX() {
        return x;
    }

    /**
     * Sets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setX(final double x) {
        this.x = x;
    }

    /**
     * Gets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getY() {
        return y;
    }

    /**
     * Sets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param y y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setY(final double y) {
        this.y = y;
    }

    /**
     * Gets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getZ() {
        return z;
    }

    /**
     * Sets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param z z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setZ(final double z) {
        this.z = z;
    }

    /**
     * Sets estimated ECEF user position coordinates.
     *
     * @param x x coordinate of estimated ECEF user position expressed in meters (m).
     * @param y y coordinate of estimated ECEF user position expressed in meters (m).
     * @param z z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setPositionCoordinates(final double x, final double y, final double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     */
    public double getAccelerationBiasX() {
        return accelerationBiasX;
    }

    /**
     * Sets estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasX(final double accelerationBiasX) {
        this.accelerationBiasX = accelerationBiasX;
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     */
    public double getAccelerationBiasY() {
        return accelerationBiasY;
    }

    /**
     * Sets estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis
     *                          and expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasY(final double accelerationBiasY) {
        this.accelerationBiasY = accelerationBiasY;
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around z axis and
     * expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasZ() {
        return accelerationBiasZ;
    }

    /**
     * Sets estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis
     *                          and expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasZ(final double accelerationBiasZ) {
        this.accelerationBiasZ = accelerationBiasZ;
    }

    /**
     * Sets estimated accelerometer bias expressed in meters per squared second (m/s^2).
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasCoordinates(
            final double accelerationBiasX, final double accelerationBiasY, final double accelerationBiasZ) {
        this.accelerationBiasX = accelerationBiasX;
        this.accelerationBiasY = accelerationBiasY;
        this.accelerationBiasZ = accelerationBiasZ;
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     */
    public double getGyroBiasX() {
        return gyroBiasX;
    }

    /**
     * Sets estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasX(final double gyroBiasX) {
        this.gyroBiasX = gyroBiasX;
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around y axis and expressed
     * in radians per second (rad/s).
     */
    public double getGyroBiasY() {
        return gyroBiasY;
    }

    /**
     * Sets estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasY estimated gyroscope bias resolved around y axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasY(final double gyroBiasY) {
        this.gyroBiasY = gyroBiasY;
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around z axis and expressed
     * in radians per second (rad/s).
     */
    public double getGyroBiasZ() {
        return gyroBiasZ;
    }

    /**
     * Sets estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasZ(final double gyroBiasZ) {
        this.gyroBiasZ = gyroBiasZ;
    }

    /**
     * Sets estimated gyroscope bias coordinates expressed in radians
     * per second (rad/s).
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis and
     *                  expressed in radians per second (rad/s).
     * @param gyroBiasY estimated gyroscope bias resolved around y axis and
     *                  expressed in radians per second (rad/s).
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasCoordinates(final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ) {
        this.gyroBiasX = gyroBiasX;
        this.gyroBiasY = gyroBiasY;
        this.gyroBiasZ = gyroBiasZ;
    }

    /**
     * Gets Kalman filter error covariance matrix.
     * Notice that covariance is expressed in terms of ECEF coordinates.
     * If accuracy of position, attitude or velocity needs to be expressed in terms
     * of NED coordinates, their respective sub-matrices of this covariance matrix
     * must be rotated, taking into account the Jacobian of the matrix transformation
     * relating both coordinates, the covariance can be expressed following the law
     * of propagation of uncertainties
     * <a href="https://en.wikipedia.org/wiki/Propagation_of_uncertainty">(https://en.wikipedia.org/wiki/Propagation_of_uncertainty)</a>
     * as: cov(f(x)) = J*cov(x)*J'.
     *
     * @param result instance where result data will be copied to.
     * @return true if result data has been copied, false otherwise.
     */
    public boolean getCovariance(final Matrix result) {
        if (covariance != null) {
            covariance.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets Kalman filter error covariance matrix.
     * Notice that covariance is expressed in terms of ECEF coordinates.
     * If accuracy of position, attitude or velocity needs to be expressed in terms
     * of NED coordinates, their respective sub-matrices of this covariance matrix
     * must be rotated, taking into account the Jacobian of the matrix transformation
     * relating both coordinates, the covariance can be expressed following the law
     * of propagation of uncertainties
     * <a href="https://en.wikipedia.org/wiki/Propagation_of_uncertainty">(https://en.wikipedia.org/wiki/Propagation_of_uncertainty)</a>
     * as: cov(f(x)) = J*cov(x)*J'.
     *
     * @return Kalman filter error covariance matrix.
     */
    public Matrix getCovariance() {
        return covariance;
    }

    /**
     * Sets Kalman filter error covariance matrix.
     *
     * @param covariance Kalman filter error covariance matrix to be set.
     * @throws IllegalArgumentException if provided covariance matrix is not 15x15.
     */
    public void setCovariance(final Matrix covariance) {
        if (covariance.getRows() != NUM_PARAMS || covariance.getColumns() != NUM_PARAMS) {
            throw new IllegalArgumentException();
        }

        this.covariance = covariance;
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @return body to ECEF coordinate transformation.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix is
     *                                        not valid (is not a 3x3 orthonormal matrix).
     */
    public CoordinateTransformation getC() throws InvalidRotationMatrixException {
        if (bodyToEcefCoordinateTransformationMatrix != null) {
            try {
                // Make sure that matrix is orthonormal
                final var fixed = fixRotationMatrix();
                return new CoordinateTransformation(fixed, FrameType.BODY_FRAME,
                        FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            } catch (final AlgebraException ignore) {
                return null;
            }

        } else {
            return null;
        }
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param threshold threshold to determine whether current body to ECEF transformation
     *                  matrix is valid or not (to check that matrix is 3x3 orthonormal).
     * @return body to ECEF coordinate transformation.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is considered not valid (is not a 3x3 orthonormal matrix) with provided threshold.
     */
    public CoordinateTransformation getC(final double threshold) throws InvalidRotationMatrixException {
        return bodyToEcefCoordinateTransformationMatrix != null
                ? new CoordinateTransformation(bodyToEcefCoordinateTransformationMatrix, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, threshold)
                : null;
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param result instance where body to ECEF coordinate transformation will be stored.
     * @return true if result instance was updated, false otherwise.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is not valid (is not a 3x3 orthonormal matrix).
     */
    public boolean getC(final CoordinateTransformation result) throws InvalidRotationMatrixException {
        if (bodyToEcefCoordinateTransformationMatrix != null) {
            result.setSourceType(FrameType.BODY_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(bodyToEcefCoordinateTransformationMatrix);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param result    instance where body to ECEF coordinate transformation will be stored.
     * @param threshold threshold to determine whether current body to ECEF transformation
     *                  matrix is valid or not (to check that matrix is 3x3 orthonormal).
     * @return true if result instance was updated, false otherwise.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is not valid (is not a 3x3 orthonormal matrix) with provided threshold.
     */
    public boolean getC(final CoordinateTransformation result, final double threshold)
            throws InvalidRotationMatrixException {
        if (bodyToEcefCoordinateTransformationMatrix != null) {
            result.setSourceType(FrameType.BODY_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(bodyToEcefCoordinateTransformationMatrix, threshold);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets body to ECEF coordinate transformation.
     *
     * @param c body to ECEF coordinate transformation to be set.
     * @throws IllegalArgumentException if provided coordinate transformation is
     *                                  not null and is not a body to ECEF transformation.
     */
    public void setC(final CoordinateTransformation c) {
        if (c == null) {
            bodyToEcefCoordinateTransformationMatrix = null;

        } else {

            if (c.getSourceType() != FrameType.BODY_FRAME
                    || c.getDestinationType() != FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME) {
                throw new IllegalArgumentException();
            }

            if (bodyToEcefCoordinateTransformationMatrix != null) {
                c.getMatrix(bodyToEcefCoordinateTransformationMatrix);
            } else {
                bodyToEcefCoordinateTransformationMatrix = c.getMatrix();
            }
        }
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around x axis will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(vx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis.
     *
     * @return estimated ECEF user velocity resolved around x axis.
     */
    public Speed getSpeedX() {
        return new Speed(vx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around x axis.
     *
     * @param vx estimated ECEF user velocity resolved around x axis.
     */
    public void setSpeedX(final Speed vx) {
        this.vx = SpeedConverter.convert(vx.getValue().doubleValue(), vx.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around y axis will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(vy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis.
     *
     * @return estimated ECEF velocity resolved around y axis.
     */
    public Speed getSpeedY() {
        return new Speed(vy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around y axis.
     *
     * @param vy estimated ECEF user velocity resolved around y axis.
     */
    public void setSpeedY(final Speed vy) {
        this.vy = SpeedConverter.convert(vy.getValue().doubleValue(), vy.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around z axis will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(vz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis.
     *
     * @return estimated ECEF velocity resolved around z axis.
     */
    public Speed getSpeedZ() {
        return new Speed(vz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around z axis.
     *
     * @param vz estimated ECEF velocity resolved around z axis.
     */
    public void setSpeedZ(final Speed vz) {
        this.vz = SpeedConverter.convert(vz.getValue().doubleValue(), vz.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param vx estimated ECEF velocity resolved around x axis.
     * @param vy estimated ECEF velocity resolved around y axis.
     * @param vz estimated ECEF velocity resolved around z axis.
     */
    public void setVelocityCoordinates(final Speed vx, final Speed vy, final Speed vz) {
        setSpeedX(vx);
        setSpeedY(vy);
        setSpeedZ(vz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @param result instance where estimated ECEF user velocity will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(vx, vy, vz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @return estimated ECEF user velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(vx, vy, vz);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param ecefVelocity estimated ECEF user velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        vx = ecefVelocity.getVx();
        vy = ecefVelocity.getVy();
        vz = ecefVelocity.getVz();
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @param result instance where x coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(x);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @return x coordinate of estimated ECEF user position.
     */
    public Distance getDistanceX() {
        return new Distance(x, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setDistanceX(final Distance x) {
        this.x = DistanceConverter.convert(x.getValue().doubleValue(), x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @param result instance where y coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(y);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public Distance getDistanceY() {
        return new Distance(y, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of estimated ECEF user position.
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setDistanceY(final Distance y) {
        this.y = DistanceConverter.convert(y.getValue().doubleValue(), y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @param result instance where z coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(z);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public Distance getDistanceZ() {
        return new Distance(z, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of estimated ECEF user position.
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setDistanceZ(final Distance z) {
        this.z = DistanceConverter.convert(z.getValue().doubleValue(), z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets coordinates of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     * @param y y coordinate of estimated ECEF user position.
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setPositionCoordinates(final Distance x, final Distance y, final Distance z) {
        setDistanceX(x);
        setDistanceY(y);
        setDistanceZ(z);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @param result instance where estimated ECEF user position expressed
     *               in meters (m) will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(x, y, z);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @return estimated ECEF user position expressed in meters (m).
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(x, y, z);
    }

    /**
     * Sets estimated ECEF user position expressed in meters (m).
     *
     * @param position estimated ECEF user position expressed in
     *                 meters (m).
     */
    public void setPosition(final Point3D position) {
        x = position.getInhomX();
        y = position.getInhomY();
        z = position.getInhomZ();
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @param result instance where estimated ECEF user position
     *               will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(x, y, z);
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @return estimated ECEF user position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(x, y, z);
    }

    /**
     * Sets estimated ECEF user position.
     *
     * @param ecefPosition estimated ECEF user position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        x = ecefPosition.getX();
        y = ecefPosition.getY();
        z = ecefPosition.getZ();
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @param result instance where estimated ECEF user position and velocity
     *               will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(x, y, z);
        result.setVelocityCoordinates(vx, vy, vz);
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @return estimated ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(x, y, z, vx, vy, vz);
    }

    /**
     * Sets estimated ECEF user position and velocity.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     */
    public void setPositionAndVelocity(final ECEFPositionAndVelocity positionAndVelocity) {
        x = positionAndVelocity.getX();
        y = positionAndVelocity.getY();
        z = positionAndVelocity.getZ();
        vx = positionAndVelocity.getVx();
        vy = positionAndVelocity.getVy();
        vz = positionAndVelocity.getVz();
    }

    /**
     * Gets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @param result instance where body to ECEF frame will be stored.
     * @return true if result was updated, false otherwise.
     */
    public boolean getFrame(final ECEFFrame result) {
        if (bodyToEcefCoordinateTransformationMatrix != null) {
            try {
                result.setCoordinateTransformation(getC());
            } catch (final InvalidSourceAndDestinationFrameTypeException | InvalidRotationMatrixException e) {
                return false;
            }
            result.setCoordinates(x, y, z);
            result.setVelocityCoordinates(vx, vy, vz);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @return body to ECEF frame.
     */
    public ECEFFrame getFrame() {
        if (bodyToEcefCoordinateTransformationMatrix != null) {
            try {
                return new ECEFFrame(x, y, z, vx, vy, vz, getC());
            } catch (final InvalidSourceAndDestinationFrameTypeException | InvalidRotationMatrixException e) {
                return null;
            }
        } else {
            return null;
        }
    }

    /**
     * Sets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @param frame body to ECEF frame to be set.
     */
    public void setFrame(final ECEFFrame frame) {
        x = frame.getX();
        y = frame.getY();
        z = frame.getZ();

        vx = frame.getVx();
        vy = frame.getVy();
        vz = frame.getVz();

        if (bodyToEcefCoordinateTransformationMatrix != null) {
            frame.getCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        } else {
            bodyToEcefCoordinateTransformationMatrix = frame.getCoordinateTransformationMatrix();
        }
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               x axis will be stored.
     */
    public void getAccelerationBiasXAsAcceleration(final Acceleration result) {
        result.setValue(accelerationBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis.
     *
     * @return estimated accelerometer bias resolved around x axis.
     */
    public Acceleration getAccelerationBiasXAsAcceleration() {
        return new Acceleration(accelerationBiasX, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around x axis.
     *
     * @param accelerationBiasX estimated accelerometer bias resolved
     *                          around x axis.
     */
    public void setAccelerationBiasX(final Acceleration accelerationBiasX) {
        this.accelerationBiasX = AccelerationConverter.convert(accelerationBiasX.getValue().doubleValue(),
                accelerationBiasX.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               y axis will be stored.
     */
    public void getAccelerationBiasYAsAcceleration(final Acceleration result) {
        result.setValue(accelerationBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis.
     *
     * @return estimated accelerometer bias resolved around y axis.
     */
    public Acceleration getAccelerationBiasYAsAcceleration() {
        return new Acceleration(accelerationBiasY, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around y axis.
     *
     * @param accelerationBiasY estimated accelerometer bias resolved
     *                          around y axis.
     */
    public void setAccelerationBiasY(final Acceleration accelerationBiasY) {
        this.accelerationBiasY = AccelerationConverter.convert(accelerationBiasY.getValue().doubleValue(),
                accelerationBiasY.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               z axis will be stored.
     */
    public void getAccelerationBiasZAsAcceleration(final Acceleration result) {
        result.setValue(accelerationBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis.
     *
     * @return estimated accelerometer bias resolved around z axis.
     */
    public Acceleration getAccelerationBiasZAsAcceleration() {
        return new Acceleration(accelerationBiasZ, AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around z axis.
     *
     * @param accelerationBiasZ estimated accelerometer bias resolved
     *                          around z axis.
     */
    public void setAccelerationBiasZ(final Acceleration accelerationBiasZ) {
        this.accelerationBiasZ = AccelerationConverter.convert(accelerationBiasZ.getValue().doubleValue(),
                accelerationBiasZ.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias coordinates.
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     */
    public void setAccelerationBiasCoordinates(
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ) {
        setAccelerationBiasX(accelerationBiasX);
        setAccelerationBiasY(accelerationBiasY);
        setAccelerationBiasZ(accelerationBiasZ);
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis.
     *
     * @param result instance where estimated gyroscope bias resolved around x axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasX(final AngularSpeed result) {
        result.setValue(gyroBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis.
     *
     * @return estimated gyroscope bias resolved around x axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasX() {
        return new AngularSpeed(gyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around x axis.
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis.
     */
    public void setGyroBiasX(final AngularSpeed gyroBiasX) {
        this.gyroBiasX = AngularSpeedConverter.convert(gyroBiasX.getValue().doubleValue(), gyroBiasX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis.
     *
     * @param result instance where estimated gyroscope bias resolved around y axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasY(final AngularSpeed result) {
        result.setValue(gyroBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis.
     *
     * @return estimated gyroscope bias resolved around y axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasY() {
        return new AngularSpeed(gyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around y axis.
     *
     * @param gyroBiasY estimated gyroscope bias resolved around y axis.
     */
    public void setGyroBiasY(final AngularSpeed gyroBiasY) {
        this.gyroBiasY = AngularSpeedConverter.convert(gyroBiasY.getValue().doubleValue(), gyroBiasY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis.
     *
     * @param result instance where estimated gyroscope bias resolved around z axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasZ(final AngularSpeed result) {
        result.setValue(gyroBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis.
     *
     * @return estimated gyroscope bias resolved around z axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasZ() {
        return new AngularSpeed(gyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around z axis.
     *
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis.
     */
    public void setGyroBiasZ(final AngularSpeed gyroBiasZ) {
        this.gyroBiasZ = AngularSpeedConverter.convert(gyroBiasZ.getValue().doubleValue(), gyroBiasZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias coordinates.
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis.
     */
    public void setGyroBiasCoordinates(
            final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY, final AngularSpeed gyroBiasZ) {
        setGyroBiasX(gyroBiasX);
        setGyroBiasY(gyroBiasY);
        setGyroBiasZ(gyroBiasZ);
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final INSLooselyCoupledKalmanState output) {
        output.copyFrom(this);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final INSLooselyCoupledKalmanState input) {
        // copy coordinate transformation matrix
        if (input.bodyToEcefCoordinateTransformationMatrix == null) {
            bodyToEcefCoordinateTransformationMatrix = null;
        } else {
            if (bodyToEcefCoordinateTransformationMatrix == null) {
                bodyToEcefCoordinateTransformationMatrix = new Matrix(input.bodyToEcefCoordinateTransformationMatrix);
            } else {
                bodyToEcefCoordinateTransformationMatrix.copyFrom(input.bodyToEcefCoordinateTransformationMatrix);
            }
        }

        vx = input.vx;
        vy = input.vy;
        vz = input.vz;

        x = input.x;
        y = input.y;
        z = input.z;

        accelerationBiasX = input.accelerationBiasX;
        accelerationBiasY = input.accelerationBiasY;
        accelerationBiasZ = input.accelerationBiasZ;

        gyroBiasX = input.gyroBiasX;
        gyroBiasY = input.gyroBiasY;
        gyroBiasZ = input.gyroBiasZ;

        // copy covariance
        if (input.covariance == null) {
            covariance = null;
        } else {
            if (covariance == null) {
                covariance = new Matrix(input.covariance);
            } else {
                covariance.copyFrom(input.covariance);
            }
        }
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(bodyToEcefCoordinateTransformationMatrix, vx, vy, vz, x, y, z,
                accelerationBiasX, accelerationBiasY, accelerationBiasZ, gyroBiasX, gyroBiasY, gyroBiasZ,
                covariance);
    }

    /**
     * Checks if provided object is a INSLooselyCoupledKalmanState having exactly the same
     * contents as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(final Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        final var other = (INSLooselyCoupledKalmanState) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final INSLooselyCoupledKalmanState other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final INSLooselyCoupledKalmanState other, final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(vx - other.vx) <= threshold
                && Math.abs(vy - other.vy) <= threshold
                && Math.abs(vz - other.vz) <= threshold
                && Math.abs(x - other.x) <= threshold
                && Math.abs(y - other.y) <= threshold
                && Math.abs(z - other.z) <= threshold
                && Math.abs(accelerationBiasX - other.accelerationBiasX) <= threshold
                && Math.abs(accelerationBiasY - other.accelerationBiasY) <= threshold
                && Math.abs(accelerationBiasZ - other.accelerationBiasZ) <= threshold
                && Math.abs(gyroBiasX - other.gyroBiasX) <= threshold
                && Math.abs(gyroBiasY - other.gyroBiasY) <= threshold
                && Math.abs(gyroBiasZ - other.gyroBiasZ) <= threshold
                && other.bodyToEcefCoordinateTransformationMatrix != null
                && other.bodyToEcefCoordinateTransformationMatrix.equals(bodyToEcefCoordinateTransformationMatrix,
                threshold) && other.covariance != null && other.covariance.equals(covariance, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final var result = (INSLooselyCoupledKalmanState) super.clone();
        copyTo(result);
        return result;
    }

    /**
     * Fixes current body to ECEF coordinate transformation matrix to ensure it
     * remains orthonormal and valid to build a Coordinate transformation or
     * a rotation.
     *
     * @return a fixed rotation matrix.
     * @throws AlgebraException if there are numerical instabilities.
     */
    private Matrix fixRotationMatrix() throws AlgebraException {
        final var decomposer = new SingularValueDecomposer(bodyToEcefCoordinateTransformationMatrix);
        decomposer.decompose();

        // fixed = u * w * v'
        final var u = decomposer.getU();
        // make sure all singular values are 1
        final var w = Matrix.identity(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS);
        final var v = decomposer.getV();

        // fixed = u * w * v'
        u.multiply(w);
        v.transpose();
        u.multiply(v);

        return u;
    }
}
