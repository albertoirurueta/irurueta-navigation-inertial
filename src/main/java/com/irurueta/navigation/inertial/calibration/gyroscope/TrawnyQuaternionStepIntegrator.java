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
import com.irurueta.numerical.ExponentialMatrixEstimator;

/**
 * Computes an integration step of a quaternion using Suh's method.
 * More information available here:
 * Trawny, N. "Indirect Kalman Filter for 3D Attitude Estimation". 2005.
 */
public class TrawnyQuaternionStepIntegrator extends QuaternionStepIntegrator {

    /**
     * Estimates exponential of a square matrix.
     */
    private final ExponentialMatrixEstimator exponentialMatrixEstimator =
            new ExponentialMatrixEstimator();

    /**
     * Initial attitude to be reused.
     */
    private Matrix quat;

    /**
     * Angular speed at initial timestamp t0 to be reused.
     */
    private Matrix omega0;

    /**
     * Angular speed at end timestamp t1 to be reused.
     */
    private Matrix omega1;

    /**
     * Skew matrix of omega0 to be reused.
     */
    private Matrix omegaSkew0;

    /**
     * Skew matrix of omega1 to be reused.
     */
    private Matrix omegaSkew1;

    /**
     * Average skew of angular speeds omega0 and omega1 to be reused.
     */
    private Matrix omegaAvgOmega;

    /**
     * Derivative of angular speed to be reused.
     */
    private Matrix dotOmega;

    /**
     * Skew matrix of derivative of angular speed to be reused.
     */
    private Matrix omegaSkewDotOmega;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix a;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix b;

    /**
     * Temporary matrix to be reused.
     */
    private Matrix tmp;

    /**
     * Exponential matrix to be reused.
     */
    private Matrix expA;

    /**
     * Instance where result of integration is stored in matrix form being reused.
     */
    private Matrix quatResult;

    /**
     * Constructor.
     * Initializes matrices being reused.
     */
    public TrawnyQuaternionStepIntegrator() {
        try {
            quat = new Matrix(Quaternion.N_PARAMS, 1);
            omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            omegaAvgOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            dotOmega = new Matrix(Quaternion.INHOM_COORDS, 1);
            omegaSkewDotOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            b = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            expA = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
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
        return QuaternionStepIntegratorType.TRAWNY;
    }

    /**
     * Performs Trawny's integration step.
     * More information available here:
     * Trawny, N. "Indirect Kalman Filter for 3D Attitude Estimation". 2005.
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
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result) throws RotationException {
        integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                currentWx, currentWy, currentWz, dt, result, exponentialMatrixEstimator, quat,
                omega0, omega1, omegaSkew0, omegaSkew1, omegaAvgOmega, dotOmega,
                omegaSkewDotOmega, a, b, tmp, expA, quatResult);
    }

    /**
     * Performs Trawny's integration step.
     * More information available here:
     * Trawny, N. "Indirect Kalman Filter for 3D Attitude Estimation". 2005.
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
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result) throws RotationException {
        try {
            final ExponentialMatrixEstimator exponentialMatrixEstimator =
                    new ExponentialMatrixEstimator();
            final Matrix quat = new Matrix(Quaternion.N_PARAMS, 1);
            final Matrix omega0 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omega1 = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omegaSkew0 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaSkew1 = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix omegaAvgOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix dotOmega = new Matrix(Quaternion.INHOM_COORDS, 1);
            final Matrix omegaSkewDotOmega = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix a = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix b = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix tmp = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix expA = new Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS);
            final Matrix quatResult = new Matrix(Quaternion.N_PARAMS, 1);
            integrationStep(initialAttitude, initialWx, initialWy, initialWz,
                    currentWx, currentWy, currentWz, dt, result, exponentialMatrixEstimator, quat,
                    omega0, omega1, omegaSkew0, omegaSkew1, omegaAvgOmega, dotOmega,
                    omegaSkewDotOmega, a, b, tmp, expA, quatResult);
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
     * @param omega0            angular speed at initial timestamp t0 to be reused. Must be 3x1.
     * @param omega1            angular speed at end timestamp t1 to be reused. Must be 3x1.
     * @param omegaSkew0        skew matrix of omega0 to be reused. Must be 4x4.
     * @param omegaSkew1        skew matrix of omega1 to be reused. Must be 4x4.
     * @param omegaAvgOmega     average skew of angular speeds omega0 and omega1 to be reused. Must be 4x4.
     * @param dotOmega          derivative of angular speed to be reused. Must be 3x1.
     * @param omegaSkewDotOmega skew matrix of derivative of angular speed to be reused. Must be 4x4.
     * @param a                 temporary matrix to be reused. Must be 4x4.
     * @param b                 temporary matrix to be reused. Must be 4x4.
     * @param tmp               temporary matrix to be reused. Must be 4x4.
     * @param expA              exponential matrix to be reused. Must be 4x4.
     * @param quatResult        instance where result of integration is stored in matrix form being
     *                          reused. Must be 4x1.
     * @throws RotationException if a numerical error occurs.
     */
    private static void integrationStep(
            Quaternion initialAttitude,
            double initialWx, double initialWy, double initialWz,
            double currentWx, double currentWy, double currentWz,
            double dt, Quaternion result,
            final ExponentialMatrixEstimator exponentialMatrixEstimator, final Matrix quat,
            final Matrix omega0, final Matrix omega1, final Matrix omegaSkew0,
            final Matrix omegaSkew1, final Matrix omegaAvgOmega, final Matrix dotOmega,
            final Matrix omegaSkewDotOmega, final Matrix a, final Matrix b, final Matrix tmp,
            final Matrix expA, final Matrix quatResult) throws RotationException {
        try {
            // normalize and copy initial attitude into matrix form
            initialAttitude.normalize();
            initialAttitude.values(quat.getBuffer());

            // angular speed at initial timestamp t0
            copyAngularSpeedToMatrix(initialWx, initialWy, initialWz, omega0);

            // angular speed at end timestamp t1
            copyAngularSpeedToMatrix(currentWx, currentWy, currentWz, omega1);

            computeOmegaSkew(omega0, omegaSkew0);
            computeOmegaSkew(omega1, omegaSkew1);

            computeOmegaAvgOmega(omega0, omega1, dt, omegaSkew1, omegaAvgOmega, dotOmega,
                    omegaSkewDotOmega);

            a.copyFrom(omegaAvgOmega);
            a.multiplyByScalar(0.5 * dt);

            omegaSkew1.multiply(omegaSkew0, b);
            omegaSkew0.multiply(omegaSkew1, tmp);
            b.subtract(tmp);

            exponentialMatrixEstimator.exponential(a, expA);

            b.multiplyByScalar(dt * dt / 48.0);

            tmp.copyFrom(expA);
            tmp.add(b);

            tmp.multiply(quat, quatResult);

            result.setValues(quatResult.getBuffer());
            result.normalize();
        } catch (final AlgebraException e) {
            throw new RotationException(e);
        }
    }

    /**
     * Computes the skew antisymmetric matrix of a vector matrix containing angular speed.
     *
     * @param omega0            column vector matrix containing angular speed. Must be 3x1.
     * @param omega1            column vector matrix containing angular speed. Must be 3x1.
     * @param dt                time step expressed in seconds.
     * @param omegaSkew1        instance being reused containing the skew antisymmetric matrix. Must be 4x4.
     * @param result            instance where result must be stored. Must be 4x4.
     * @param dotOmega          instance where computed time derivative will be stored. Must be 3x1.
     * @param omegaSkewDotOmega instance where computed time derivative will be stored. Must be 4x4.
     * @throws AlgebraException if provided matrices do not have proper size.
     */
    private static void computeOmegaAvgOmega(
            final Matrix omega0, final Matrix omega1, final double dt, final Matrix omegaSkew1,
            final Matrix result, final Matrix dotOmega, final Matrix omegaSkewDotOmega)
            throws AlgebraException {
        omega1.subtract(omega0, dotOmega);
        dotOmega.multiplyByScalar(1.0 / dt);

        computeOmegaSkew(dotOmega, omegaSkewDotOmega);

        omegaSkewDotOmega.multiplyByScalar(0.5 * dt);

        omegaSkew1.add(omegaSkewDotOmega, result);
    }
}
