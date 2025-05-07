/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;

/**
 * Computes an integration step of a quaternion using Suh's method.
 * More information available here:
 * Yuan, S. "Quaternion-based Unscented Kalman Filter for Real-time". 2015,
 */
public class YuanQuaternionStepIntegrator extends QuaternionStepIntegrator {

    private static final double EPSILON = 1e-15;

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix quat;

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix omega;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix a;

    /**
     * Identity matrix to be reused.
     */
    private Matrix identity;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix tmp;

    /**
     * Instance where result of integration is stored in matrix form being reused.
     */
    private Matrix quatResult;

    /**
     * Constructor.
     * Initializes matrices being reused.
     */
    public YuanQuaternionStepIntegrator() {
        try {
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omega = new Matrix(Quaternion.N_ANGLES, 1);
            a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            quatResult = new Matrix(Quaternion.N_PARAMS, 1);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    /**
     * Gets type of this integrator.
     *
     * @return indicates type of this integrator.
     */
    @Override
    public QuaternionStepIntegratorType getType() {
        return QuaternionStepIntegratorType.YUAN;
    }

    /**
     * Performs Yuan's integration step.
     * More information available here:
     * Yuan, S. "Quaternion-based Unscented Kalman Filter for Real-time". 2015,
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).*
     * @param currentWx       end x-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWy       end y-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWz       end z-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param dt              time step expressed in seconds.
     * @param result          instance where result of integration will be stored.
     * @throws RotationException if a numerical error occurs.
     */
    @Override
    public void integrate(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result) throws RotationException {
        integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                currentWx, currentWy, currentWz, dt, result, quat, omega, a, identity, tmp,
                quatResult);
    }

    /**
     * Performs Yuan's integration step.
     * More information available here:
     * Yuan, S. "Quaternion-based Unscented Kalman Filter for Real-time". 2015,
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at initial timestamp expressed
     *                        in radians per second (rad/s).
     * @param currentWx       end x-coordinate rotation velocity at end timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWy       end y-coordinate rotation velocity at end timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWz       end z-coordinate rotation velocity at end timestamp expressed in
     *                        radians per second (rad/s).
     * @param dt              time step expressed in seconds (t1 - t0).
     * @param result          instance where result of integration will be stored.
     * @throws RotationException if a numerical error occurs.
     */
    public static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result) throws RotationException {
        try {
            final var quat = new Matrix(Quaternion.N_PARAMS, 1);
            final var omega = new Matrix(Quaternion.N_ANGLES, 1);
            final var a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, quat, omega, a, identity, tmp, quatResult);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    /**
     * Internal method computing an integration step using Trawny's algorithm.
     * This method is used internally so that reusable instances can be provided as parameters.
     *
     * @param initialAttitude   initial attitude.
     * @param initialWx         initial x-coordinate rotation velocity at initial timestamp expressed
     *                          in radians per second (rad/s).
     * @param initialWy         initial y-coordinate rotation velocity at initial timestamp expressed
     *                          in radians per second (rad/s).
     * @param initialWz         initial z-coordinate rotation velocity at initial timestamp expressed
     *                          in radians per second (rad/s).
     * @param currentWx         end x-coordinate rotation velocity at end timestamp expressed in
     *                          radians per second (rad/s).
     * @param currentWy         end y-coordinate rotation velocity at end timestamp expressed in
     *                          radians per second (rad/s).
     * @param currentWz         end z-coordinate rotation velocity at end timestamp expressed in
     *                          radians per second (rad/s).
     * @param dt                time step expressed in seconds (t1 - t0).
     * @param result            instance where result of integration will be stored.
     * @param quat              initial attitude to be reused. Must be 4x1.
     * @param omega             angular speed at initial timestamp t0 to be reused. Must be 3x1.
     * @param a                 temporary matrix to be reused. Must be 4x4.
     * @param identity          identity matrix to be reused. Must be 4x4.
     * @param tmp               temporary matrix to be reused. Must be 4x4.
     * @param quatResult        instance where result of integration is stored in matrix form being
     *                          reused. Must be 4x1.
     * @throws RotationException if a numerical error occurs.
     */
    private static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result, final Matrix quat, final Matrix omega,
            final Matrix a, final Matrix identity, final Matrix tmp, final Matrix quatResult)
            throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            final var w1 = (initialWx + currentWx) / 2.0;
            final var w2 = (initialWy + currentWy) / 2.0;
            final var w3 = (initialWz + currentWz) / 2.0;

            copyAngularSpeedToMatrix(w1, w2, w3, omega);
            computeOmegaSkew(omega, a);

            final var w1dt = w1 * dt;
            final var w2dt = w2 * dt;
            final var w3dt = w3 * dt;

            final var w1dt2 = w1dt * w1dt;
            final var w2dt2 = w2dt * w2dt;
            final var w3dt2 = w3dt * w3dt;

            final var theta = Math.sqrt(w1dt2 + w2dt2 + w3dt2);
            final var halfTheta = theta / 2;
            final double sinc;
            if (theta > EPSILON) {
                sinc = Math.sin(halfTheta) / halfTheta;
            } else {
                // notice that sin(x) / x --> 1 for x --> 0
                sinc = 1.0;
            }

            a.multiplyByScalar(0.5 * sinc * dt);

            tmp.copyFrom(identity);
            tmp.multiplyByScalar(Math.cos(halfTheta));
            tmp.add(a);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException ex) {
            throw new RotationException(ex);
        }
    }
}
