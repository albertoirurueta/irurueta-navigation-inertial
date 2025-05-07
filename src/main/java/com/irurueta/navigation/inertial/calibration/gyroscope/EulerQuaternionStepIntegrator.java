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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;

/**
 * Computes an integration step of a quaternion using Euler's method.
 * More information available here:
 * <a href="https://en.wikipedia.org/wiki/Euler_method">https://en.wikipedia.org/wiki/Euler_method</a>
 */
public class EulerQuaternionStepIntegrator extends QuaternionStepIntegrator {

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix omega0;

    /**
     * Initial attitude to be reused.
     */
    private Matrix quat;

    /**
     * Instance where result of integration is stored in matrix form being reused.
     */
    private Matrix quatResult;

    /**
     * Slope of quaternion derivative at initial timestamp t0 to be reused.
     */
    private Matrix k1;

    /**
     * Skew antisymmetric matrix used for quaternion time derivative computation to be reused.
     */
    private Matrix omegaSkew;

    /**
     * Constructor.
     * Initializes matrices being reused.
     */
    public EulerQuaternionStepIntegrator() {
        try {
            omega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            k1 = new Matrix(Quaternion.N_PARAMS, 1);
            omegaSkew = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
        return QuaternionStepIntegratorType.EULER_METHOD;
    }

    /**
     * Performs an integration step using Euler's method.
     * More information available here:
     * <a href="https://en.wikipedia.org/wiki/Euler_method">https://en.wikipedia.org/wiki/Euler_method</a>
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
        integrationStep(initialAttitude, initialWx, initialWy, initialWz, dt, result, omega0, quat, quatResult, k1,
                omegaSkew);
    }

    /**
     * Performs an integration step using Euler's method.
     * More information available here:
     * <a href="https://en.wikipedia.org/wiki/Euler_method">https://en.wikipedia.org/wiki/Euler_method</a>
     * This method should only be used sporadically. For better performance, if this method needs
     * to be called very frequently, a new instance of {@link EulerQuaternionStepIntegrator} should
     * be used and the non-static method should be used instead.
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       end x-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWy       end y-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWz       end z-coordinate rotation velocity at current timestamp expressed in
     *                        radians per second (rad/s).
     * @param dt              time step expressed in seconds.
     * @param result          instance where result of integration will be stored.
     * @throws RotationException if a numerical error occurs.
     */
    public static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double dt, final Quaternion result) throws RotationException {

        try {
            final var omega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final var quat = new Matrix(Quaternion.N_PARAMS, 1);
            final var quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            final var k1 = new Matrix(Quaternion.N_PARAMS, 1);
            final var omegaSkew = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz, dt, result, omega0, quat, quatResult, k1,
                    omegaSkew);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    /**
     * Internal method computing an integration step using Euler's method.
     * This method is used internally so that reusable instances can be provided as parameters.
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at current timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at current timestamp expressed
     *                        in radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at current timestamp expressed
     *                        in radians per second (rad/s).
     * @param dt              time step expressed in seconds.
     * @param result          instance where result of integration will be stored.
     * @param omega0          angular speed at initial timestamp t0 to be reused. Must be 3x1.
     * @param quat            initial attitude to be reused. Must be 4x1.
     * @param quatResult      instance where result of integration is stored in matrix form being
     *                        reused. Must be 4x1.
     * @param k1              slope of quaternion derivative at initial timestamp t0 to be reused.
     *                        Must be 4x1.
     * @param omegaSkew       skew antisymmetric matrix used for quaternion time derivative
     *                        computation to be reused. Must be 4x4.
     * @throws RotationException if a numerical error occurs.
     */
    private static void integrationStep(
            final Quaternion initialAttitude, final double initialWx, final double initialWy, final double initialWz,
            final double dt, final Quaternion result, final Matrix omega0, final Matrix quat,
            final Matrix quatResult, final Matrix k1, final Matrix omegaSkew) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // Compute slope k1 at initial point: k1 = f(t(n), x(n))
            computeOmegaSkew(omega0, omegaSkew);
            computeTimeDerivative(quat, omegaSkew, k1);

            // Euler method follows expression:
            // x(n + 1) = x(n) + dt * k1
            k1.multiplyByScalar(dt);
            quatResult.copyFrom(k1);
            quatResult.add(quat);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }
}
