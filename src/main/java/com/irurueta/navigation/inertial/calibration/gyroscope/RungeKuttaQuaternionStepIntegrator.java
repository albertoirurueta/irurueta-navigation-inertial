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
 * Computes one step of a Runge-Kutta (RK4) integration algorithm.
 * More information available here:
 * <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods">
 *     https://en.wikipedia.org/wiki/Runge–Kutta_methods
 * </a>
 */
public class RungeKuttaQuaternionStepIntegrator extends QuaternionStepIntegrator {

    /**
     * Factor to multiply slopes k1 and k4.
     */
    private static final double ONE_SIXTH = 1.0 / 6.0;

    /**
     * Factor to multiply slopes k2 and k3.
     */
    private static final double ONE_THIRD = 1.0 / 3.0;

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix mOmega0;

    /**
     * Angular speed at end timestamp t1 to be reused.
     */
    private Matrix mOmega1;

    /**
     * Angular speed at mid-point timestamp between t0 and t1 to be reused.
     */
    private Matrix mOmega01;

    /**
     * Initial attitude to be reused.
     */
    private Matrix mQuat;

    /**
     * Instance where result of integration is stored in matrix form being reused.
     */
    private Matrix mQuatResult;

    /**
     * Temporal quaternion used to compute additional slopes that is being reused.
     */
    private Matrix mTmpQ;

    /**
     * First Runge-Kutta coefficient. Quaternion derivative at initial timestamp t0 to be
     * reused.
     */
    private Matrix mK1;

    /**
     * Second Runge-Kutta coefficient. Quaternion derivative at mid-point timestamp
     * between t0 and t1 to be reused.
     */
    private Matrix mK2;

    /**
     * Third Runge-Kutta coefficient to be reused.
     */
    private Matrix mK3;

    /**
     * Fourth Runge-Kutta coefficient. Quaternion derivative at end timestamp t1 to be
     * reused.
     */
    private Matrix mK4;

    /**
     * Skew antisymmetric matrix used for quaternion time derivative computation to be reused.
     */
    private Matrix mOmegaSkew;

    /**
     * Constructor.
     * Initializes matrices being reused.
     */
    public RungeKuttaQuaternionStepIntegrator() {
        try {
            mOmega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            mOmega1 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            mOmega01 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            mQuat = new Matrix(Quaternion.N_PARAMS, 1);
            mQuatResult = new Matrix(Quaternion.N_PARAMS, 1);
            mTmpQ = new Matrix(Quaternion.N_PARAMS, 1);
            mK1 = new Matrix(Quaternion.N_PARAMS, 1);
            mK2 = new Matrix(Quaternion.N_PARAMS, 1);
            mK3 = new Matrix(Quaternion.N_PARAMS, 1);
            mK4 = new Matrix(Quaternion.N_PARAMS, 1);
            mOmegaSkew = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
        return QuaternionStepIntegratorType.RUNGE_KUTTA;
    }

    /**
     * Performs an RK4 Runge-Kutta integration step.
     * More information available here:
     * <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods">
     *     https://en.wikipedia.org/wiki/Runge–Kutta_methods
     * </a>–
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
    @Override
    public void integrate(
            final Quaternion initialAttitude, final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz, final double dt,
            Quaternion result) throws RotationException {
        integrationStep(initialAttitude, initialWx, initialWy, initialWz, currentWx, currentWy, currentWz, dt, result,
                mOmega0, mOmega1, mOmega01, mQuat, mQuatResult, mTmpQ, mK1, mK2, mK3, mK4, mOmegaSkew);
    }

    /**
     * Performs an RK4 Runge-Kutta integration step.
     * More information available here:
     * <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods">
     *     https://en.wikipedia.org/wiki/Runge–Kutta_methods
     * </a>
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWx       end x-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWy       end y-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWz       end z-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param dt              time step expressed in seconds (t1 - t0).
     * @param result          instance where result of integration will be stored.
     * @throws RotationException if a numerical error occurs.
     */
    public static void integrationStep(
            final Quaternion initialAttitude, final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz, final double dt, Quaternion result)
            throws RotationException {
        try {
            final Matrix omega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final Matrix omega1 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final Matrix omega01 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final Matrix quat = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix tmpQ = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k1 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k2 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k3 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix k4 = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix omegaSkew = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz, currentWx, currentWy, currentWz, dt,
                    result, omega0, omega1, omega01, quat, quatResult, tmpQ, k1, k2, k3, k4, omegaSkew);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    /**
     * Performs an RK4 Runge-Kutta integration step.
     * More information available here:
     * <a href="https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods">
     *     https://en.wikipedia.org/wiki/Runge–Kutta_methods
     * </a>
     *
     * @param initialAttitude initial attitude.
     * @param initialWx       initial x-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWy       initial y-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param initialWz       initial z-coordinate rotation velocity at initial timestamp expressed in
     *                        radians per second (rad/s).
     * @param currentWx       end x-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWy       end y-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param currentWz       end z-coordinate rotation velocity at end timestamp expressed in radians
     *                        per second (rad/s).
     * @param dt              time step expressed in seconds (t1 - t0).
     * @param result          instance where result of integration will be stored.
     * @param omega0          angular speed at initial timestamp t0 to be reused. Must be 3x1.
     * @param omega1          angular speed at end timestamp t1 to be reused. Must be 3x1.
     * @param omega01         angular speed at mid-point timestamp between t0 and t1 to be reused.
     *                        Must be 3x1.
     * @param quat            initial attitude to be reused. Must be 4x1.
     * @param quatResult      instance where result of integration is stored in matrix form being
     *                        reused. Must be 4x1.
     * @param tmpQ            temporal quaternion used to compute additional slopes that is being
     *                        reused. Must be 4x1.
     * @param k1              slope of quaternion derivative at initial timestamp t0 to be reused.
     *                        Must be 4x1.
     * @param k2              slope of quaternion derivative at mid-point timestamp between t0 and
     *                        t1 to be reused. Must be 4x1.
     * @param k3              slope of quaternion derivative at mid-point timestamp between t0 and
     *                        t1 to be reused. Must be 4x1.
     * @param k4              slope of quaternion derivative at end timestamp t1 to be reused.
     *                        Must be 4x1.
     * @param omegaSkew       skew antisymmetric matrix used for quaternion time derivative
     *                        computation to be reused. Must be 4x4.
     * @throws RotationException if a numerical error occurs.
     */
    private static void integrationStep(
            final Quaternion initialAttitude, final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz, final double dt, Quaternion result,
            final Matrix omega0, final Matrix omega1, final Matrix omega01, final Matrix quat, final Matrix quatResult,
            final Matrix tmpQ, final Matrix k1, final Matrix k2, final Matrix k3, final Matrix k4,
            final Matrix omegaSkew) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // angular speed at end timestamp t1
            copyAngularSpeedToMatrix(currentWx, currentWy, currentWz, omega1);

            // compute average of angular speeds at mid-point between t0 and t1
            computeAverageAngularSpeed(initialWx, initialWy, initialWz, currentWx, currentWy, currentWz, omega01);

            // Compute First Runge-Kutta coefficient k1 as the slope at initial point: k1 = f(t(n), x(n))
            // so that x(t(n) + 0.5 * dt) = x(n) + 0.5 * dt * k1
            computeOmegaSkew(omega0, omegaSkew);
            computeTimeDerivative(quat, omegaSkew, k1);

            // Compute Second Runge-Kutta coefficient k2 as the slope at mid-point:
            // k2 = f(t(n) + 0.5 * dt, x(t(n) + 0.5 * dt))
            // k2 = f(t(n) + 0.5 * dt, x(n) + 0.5 * dt * k1)
            final double halfDt = 0.5 * dt;
            tmpQ.copyFrom(k1);
            tmpQ.multiplyByScalar(halfDt);
            tmpQ.add(quat);
            computeOmegaSkew(omega01, omegaSkew);
            computeTimeDerivative(tmpQ, omegaSkew, k2);

            // Third Runge-Kutta coefficient k3 as the slope at mid-point (same omega skew as k2):
            // k3 = f(t(n) + 0.5 * dt, x(t(n) + 0.5 * dt))
            // k3 = f(t(n) + 0.5 * dt, x(n) + 0.5 * dt * k2)
            tmpQ.copyFrom(k2);
            tmpQ.multiplyByScalar(halfDt);
            tmpQ.add(quat);
            computeTimeDerivative(tmpQ, omegaSkew, k3);

            // Fourth Runge-Kutta coefficient k4 as the slope at end-point:
            // k4 = f(t(n) + dt, x(t(n) + dt))
            // k4 = f(t(n) + dt, x(n) + dt * k3)
            tmpQ.copyFrom(k3);
            tmpQ.multiplyByScalar(dt);
            tmpQ.add(quat);
            computeOmegaSkew(omega1, omegaSkew);
            computeTimeDerivative(tmpQ, omegaSkew, k4);

            // result = quat + dt * (ONE_SIXTH * k1 + ONE_THIRD * k2 + ONE_THIRD * k3 + ONE_SIXTH * k4)
            k1.multiplyByScalar(ONE_SIXTH);
            k2.multiplyByScalar(ONE_THIRD);
            k3.multiplyByScalar(ONE_THIRD);
            k4.multiplyByScalar(ONE_SIXTH);

            quatResult.copyFrom(k1);
            quatResult.add(k2);
            quatResult.add(k3);
            quatResult.add(k4);
            quatResult.multiplyByScalar(dt);
            quatResult.add(quat);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }
}
