/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;

/**
 * Contains common methods and factory methods for all implementations of this class.
 */
public abstract class QuaternionStepIntegrator {

    /**
     * Default quaternion step integrator type.
     */
    public static final QuaternionStepIntegratorType DEFAULT_TYPE = QuaternionStepIntegratorType.RUNGE_KUTTA;

    /**
     * Gets type of this integrator.
     *
     * @return indicates type of this integrator.
     */
    public abstract QuaternionStepIntegratorType getType();

    /**
     * Performs an integration step.
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).*
     * @param currentWx       end x-coordinate rotation velocity at current timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWy       end y-coordinate rotation velocity at current timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWz       end z-coordinate rotation velocity at current timestamp expressed in radians
     *                        per second (rad/s).
     * @param dt              time step expressed in seconds.
     * @param result          instance where result of integration will be stored.
     * @throws RotationException if a numerical error occurs.
     */
    public abstract void integrate(
            final Quaternion initialAttitude, final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz, final double dt,
            final Quaternion result) throws RotationException;

    /**
     * Creates a quaternion step integrator using provided type.
     *
     * @param type type of quaternion step integrator.
     * @return created quaternion step integrator.
     */
    public static QuaternionStepIntegrator create(QuaternionStepIntegratorType type) {
        return switch (type) {
            case EULER_METHOD -> new EulerQuaternionStepIntegrator();
            case MID_POINT -> new MidPointQuaternionStepIntegrator();
            default -> new RungeKuttaQuaternionStepIntegrator();
        };
    }

    /**
     * Creates a quaternion step integrator using default type.
     *
     * @return created quaternion step integrator.
     */
    public static QuaternionStepIntegrator create() {
        return create(DEFAULT_TYPE);
    }

    /**
     * Computes the time derivative of a quaternion at a given angular speed.
     *
     * @param quaternion column vector matrix containing quaternion to compute time derivative for.
     *                   Must be 4x1.
     * @param omegaSkew  instance being reused containing the skew antisymmetric matrix. Must be 4x4.
     * @param result     instance where computed time derivative will be stored. Must be 4x1.
     * @throws WrongSizeException if provided matrices do not have proper size.
     */
    protected static void computeTimeDerivative(
            final Matrix quaternion, final Matrix omegaSkew, final Matrix result) throws WrongSizeException {

        // The time derivative of a quaternion at a given angular speed follows expression:
        // q`= 0.5 * W * q
        // where W is the skew antisymmetric matrix of provided angular speed and q is a quaternion
        // containing a given attitude.
        omegaSkew.multiply(quaternion, result);
        result.multiplyByScalar(0.5);
    }

    /**
     * Computes the skew antisymmetric matrix of a vector matrix containing angular speed.
     *
     * @param omega  column vector matrix containing angular speed. Must be 3x1.
     * @param result instance where result must be stored. Must be 4x4.
     */
    protected static void computeOmegaSkew(final Matrix omega, final Matrix result) {

        final double wx = omega.getElementAtIndex(0);
        final double wy = omega.getElementAtIndex(1);
        final double wz = omega.getElementAtIndex(2);

        // The skew matrix has the following expression:
        // W = [0		-wx		-wy		-wz]
        //     [wx		0		wz		-wy]
        //     [wy		-wz		0		 wx]
        //     [wz		wy		-wx		  0]

        result.setElementAtIndex(0, 0.0);
        result.setElementAtIndex(1, wx);
        result.setElementAtIndex(2, wy);
        result.setElementAtIndex(3, wz);

        result.setElementAtIndex(4, -wx);
        result.setElementAtIndex(5, 0.0);
        result.setElementAtIndex(6, -wz);
        result.setElementAtIndex(7, wy);

        result.setElementAtIndex(8, -wy);
        result.setElementAtIndex(9, wz);
        result.setElementAtIndex(10, 0.0);
        result.setElementAtIndex(11, -wx);

        result.setElementAtIndex(12, -wz);
        result.setElementAtIndex(13, -wy);
        result.setElementAtIndex(14, wx);
        result.setElementAtIndex(15, 0.0);
    }

    /**
     * Copies provided angular speed coordinates into a matrix.
     *
     * @param wx     x-coordinate of angular speed expressed in radians per second (rad/s).
     * @param wy     y-coordinate of angular speed expressed in radians per second (rad/s).
     * @param wz     z-coordinate of angular speed expressed in radians per second (rad/s).
     * @param result instance where angular speed coordinates are copied to.
     */
    protected static void copyAngularSpeedToMatrix(
            final double wx, final double wy, final double wz, final Matrix result) {
        result.setElementAtIndex(0, wx);
        result.setElementAtIndex(1, wy);
        result.setElementAtIndex(2, wz);
    }

    /**
     * Computes average angular speed into matrix form at mid-point between initial timestamp t0
     * and end timestamp t1.
     *
     * @param initialWx initial x-coordinate rotation velocity at initial timestamp expressed
     *                  in radians per second (rad/s).
     * @param initialWy initial y-coordinate rotation velocity at initial timestamp expressed
     *                  in radians per second (rad/s).
     * @param initialWz initial z-coordinate rotation velocity at initial timestamp expressed
     *                  in radians per second (rad/s).*
     * @param currentWx end x-coordinate rotation velocity at current timestamp expressed in
     *                  radians per second (rad/s).
     * @param currentWy end y-coordinate rotation velocity at current timestamp expressed in
     *                  radians per second (rad/s).
     * @param currentWz end z-coordinate rotation velocity at current timestamp expressed in
     *                  radians per second (rad/s).
     * @param result    instance where result will be stored. Must be 3x1.
     */
    protected static void computeAverageAngularSpeed(
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz, final Matrix result) {
        result.setElementAtIndex(0, 0.5 * (initialWx + currentWx));
        result.setElementAtIndex(1, 0.5 * (initialWy + currentWy));
        result.setElementAtIndex(2, 0.5 * (initialWz + currentWz));
    }
}
