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
import com.irurueta.algebra.FrobeniusNormComputer;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;

/**
 * Computes an integration step of a quaternion using Suh's method.
 * More information available here:
 * Young Soo Suh. "Orientation estimation using a quaternion-based indirect Kalman filter with adaptive estimation of
 * external acceleration". 2010.
 */
public class SuhQuaternionStepIntegrator extends QuaternionStepIntegrator {

    /**
     * Precomputed 3/4 factor
     */
    private static final double THREE_FOURTHS = 3.0 / 4.0;

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix omega0;

    /**
     * Angular speed at end timestamp t1 to be reused.
     */
    private Matrix omega1;

    /**
     * Initial attitude to be reused.
     */
    private Matrix quat;

    /**
     * Skew matrix of omega0 to be reused.
     */
    private Matrix omegaSkew0;

    /**
     * Skew matrix of omega1 to be reused.
     */
    private Matrix omegaSkew1;

    /**
     * Constant matrix to be reused.
     */
    private Matrix constant;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix omegaSkew10;

    /**
     * Identity matrix to be reused.
     */
    private Matrix identity;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix omegaSkew1A;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix omegaSkew0A;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix omegaSkew1B;

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
    public SuhQuaternionStepIntegrator() {
        try {
            omega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            omega1 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            constant = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew10 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew0A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1B = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
        return QuaternionStepIntegratorType.SUH;
    }

    /**
     * Performs Suh's integration step.
     * More information available here:
     * Young Soo Suh. "Orientation estimation using a quaternion-based indirect Kalman filter with adaptive estimation
     * of external acceleration". 2010.
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
                currentWx, currentWy, currentWz, dt, result, omega0, omega1, quat, omegaSkew0,
                omegaSkew1, constant, omegaSkew10, identity, omegaSkew1A, omegaSkew0A,
                omegaSkew1B, tmp, quatResult);
    }

    /**
     * Performs Suh's integration step.
     * More information available here:
     * Young Soo Suh. "Orientation estimation using a quaternion-based indirect Kalman filter with adaptive estimation
     * of external acceleration". 2010.
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
            final var omega0 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final var omega1 = new Matrix(Rotation3D.INHOM_COORDS, 1);
            final var quat = new Matrix(Quaternion.N_PARAMS, 1);
            final var omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var constant = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var omegaSkew10 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var identity = Matrix.identity(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var omegaSkew1A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var omegaSkew0A = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var omegaSkew1B = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final var quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, omega0, omega1, quat, omegaSkew0,
                    omegaSkew1, constant, omegaSkew10, identity, omegaSkew1A, omegaSkew0A,
                    omegaSkew1B, tmp, quatResult);
        } catch (final AlgebraException ignore) {
            // never happens
        }
    }

    /**
     * Internal method computing an integration step using Suh's algorithm.
     * This method is used internally so that reusable instances can be provided as parameters.
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
     * @param omega0          angular speed at initial timestamp t0 to be reused. Must be 3x1.
     * @param omega1          angular speed at end timestamp t1 to be reused. Must be 3x1.
     * @param quat            initial attitude to be reused. Must be 4x1.
     * @param omegaSkew0      skew matrix of omega0 to be reused. Must be 4x4.
     * @param omegaSkew1      skew matrix of omega1 to be reused. Must be 4x4.
     * @param constant        constant matrix to be reused. Must be 4x4.
     * @param omegaSkew10     temporary matrix to be reused. Must be 4x4.
     * @param identity        identity matrix to be reused. Must be 4x4.
     * @param omegaSkew1A     temporary matrix to be reused. Must be 4x4.
     * @param omegaSkew0A     temporary matrix to be reused. Must be 4x4.
     * @param omegaSkew1B     temporary matrix to be reused. Must be 4x4.
     * @param tmp             temporary matrix to be reused. Must be 4x4.
     * @param quatResult      instance where result of integration is stored in matrix form being
     *                        reused. Must be 4x1.
     * @throws RotationException if a numerical error occurs.
     */
    private static void integrationStep(
            final Quaternion initialAttitude,
            final double initialWx, final double initialWy, final double initialWz,
            final double currentWx, final double currentWy, final double currentWz,
            final double dt, final Quaternion result, final Matrix omega0, final Matrix omega1,
            final Matrix quat, final Matrix omegaSkew0, final Matrix omegaSkew1,
            final Matrix constant, final Matrix omegaSkew10, final Matrix identity,
            final Matrix omegaSkew1A, final Matrix omegaSkew0A, final Matrix omegaSkew1B,
            final Matrix tmp, final Matrix quatResult) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // angular speed at end timestamp t1
            copyAngularSpeedToMatrix(currentWx, currentWy, currentWz, omega1);

            final var norm1 = FrobeniusNormComputer.norm(omega1);
            final var sqrNorm1 = norm1 * norm1;

            computeOmegaSkew(omega0, omegaSkew0);
            computeOmegaSkew(omega1, omegaSkew1);

            final var dt2 = dt * dt;
            final var dt3 = dt * dt2;

            constant.initialize(sqrNorm1 * dt2 / 6.0);

            omegaSkew1.multiply(omegaSkew0, omegaSkew10);
            omegaSkew10.multiplyByScalar(dt2 / 24.0);

            omegaSkew1A.copyFrom(omegaSkew1);
            omegaSkew1A.multiplyByScalar(THREE_FOURTHS * dt);

            omegaSkew0A.copyFrom(omegaSkew0);
            omegaSkew0A.multiplyByScalar(dt / 4.0);

            omegaSkew1B.copyFrom(omegaSkew1);
            omegaSkew1B.multiplyByScalar(sqrNorm1 * dt3 / 48.0);

            tmp.copyFrom(identity);
            tmp.add(omegaSkew1A);
            tmp.subtract(omegaSkew0A);
            tmp.subtract(constant);
            tmp.subtract(omegaSkew10);
            tmp.subtract(omegaSkew1B);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }
}
