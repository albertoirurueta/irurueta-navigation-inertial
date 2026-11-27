/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Checks that the general-case gyroscope calibrators propagate the fitter
 * covariance to the published parameter covariance correctly.
 * <p>
 * The fitter unknowns are b = (bx, by, bz) (only when biases are estimated),
 * M = Mg + I stored column-wise (m11, m21, m31, m12, m22, m32, m13, m23, m33)
 * and G stored column-wise (only when G-dependent cross biases are estimated).
 * The published parameters are:
 * <pre>
 * bg = M * b                              (only when biases are estimated)
 * sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy = entries of M - I
 * Gg = M * G                              (zero when cross biases are not estimated)
 * </pre>
 * This class rebuilds that mapping independently of the calibrators,
 * differentiates it numerically, and checks that J * Cov * J' equals the
 * covariance published by the calibrator. A wrong or misplaced entry anywhere
 * in a calibrator's analytical Jacobian makes the check fail.
 */
final class GyroscopeCovariancePropagationAssert {

    /**
     * Number of components of a 3D vector.
     */
    private static final int COMPONENTS = 3;

    /**
     * Number of entries of a 3x3 matrix.
     */
    private static final int MATRIX_ENTRIES = 9;

    /**
     * Position of sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy within the column-wise
     * M block of fitter unknowns.
     */
    private static final int[] M_INDICES = {0, 4, 8, 3, 6, 1, 7, 2, 5};

    /**
     * Step used for central differences. The mapping is bilinear, so central
     * differences are exact up to rounding error for any step.
     */
    private static final double STEP = 1e-3;

    /**
     * Tolerance relative to sqrt(var_i * var_j) for covariance entry (i, j).
     */
    private static final double RELATIVE_ERROR = 1e-8;

    private GyroscopeCovariancePropagationAssert() {
    }

    /**
     * Asserts that the covariance published by a calibrator after a general-case
     * calibration equals the fitter covariance propagated through a numerically
     * differentiated Jacobian.
     *
     * @param calibrator           calibrator that has finished a general-case calibration.
     * @param estimatedCovariance  covariance returned by the calibrator.
     * @param estimatesBias        true when the calibrator estimates biases (bias unknowns
     *                             and published bg rows are present), false for known-bias
     *                             calibrators.
     * @param estimatesCrossBiases true when G-dependent cross biases are estimated.
     */
    static void assertCovariancePropagated(
            final Object calibrator, final Matrix estimatedCovariance, final boolean estimatesBias,
            final boolean estimatesCrossBiases) {
        final var fitter = getFitter(calibrator);
        final var params = getParams(fitter);
        final var fitterCovariance = getCovariance(fitter);

        final var inputs = (estimatesBias ? COMPONENTS : 0) + MATRIX_ENTRIES
                + (estimatesCrossBiases ? MATRIX_ENTRIES : 0);
        final var outputs = (estimatesBias ? COMPONENTS : 0) + 2 * MATRIX_ENTRIES;
        assertEquals(inputs, params.length);
        assertEquals(inputs, fitterCovariance.getRows());
        assertEquals(outputs, estimatedCovariance.getRows());
        assertEquals(outputs, estimatedCovariance.getColumns());

        final var jacobian = numericalJacobian(params, outputs, estimatesBias, estimatesCrossBiases);
        final var expected = propagate(jacobian, fitterCovariance);

        for (var i = 0; i < outputs; i++) {
            for (var j = 0; j < outputs; j++) {
                final var tolerance = RELATIVE_ERROR * Math.sqrt(Math.abs(expected[i][i] * expected[j][j]));
                assertEquals(expected[i][j], estimatedCovariance.getElementAt(i, j), tolerance,
                        "covariance entry (" + i + ", " + j + ")");
            }
        }

        // mzy variance must be non-trivial so that a scaled Jacobian entry
        // cannot go unnoticed
        final var mzyIndex = (estimatesBias ? COMPONENTS : 0) + MATRIX_ENTRIES - 1;
        assertTrue(estimatedCovariance.getElementAt(mzyIndex, mzyIndex) > 0.0);
    }

    /**
     * Evaluates the mapping from fitter unknowns to published parameters.
     */
    private static double[] map(final double[] x, final int outputs, final boolean estimatesBias,
                                final boolean estimatesCrossBiases) {
        final var mOffset = estimatesBias ? COMPONENTS : 0;
        final var gOffset = mOffset + MATRIX_ENTRIES;
        final var result = new double[outputs];
        var pos = 0;

        if (estimatesBias) {
            // bg = M * b
            for (var row = 0; row < COMPONENTS; row++) {
                var value = 0.0;
                for (var k = 0; k < COMPONENTS; k++) {
                    value += m(x, mOffset, row, k) * x[k];
                }
                result[pos++] = value;
            }
        }

        // entries of Mg = M - I
        for (var i = 0; i < MATRIX_ENTRIES; i++) {
            final var index = M_INDICES[i];
            final var diagonal = index % (COMPONENTS + 1) == 0;
            result[pos++] = x[mOffset + index] - (diagonal ? 1.0 : 0.0);
        }

        // Gg = M * G stored column-wise, or zero
        for (var col = 0; col < COMPONENTS; col++) {
            for (var row = 0; row < COMPONENTS; row++) {
                var value = 0.0;
                if (estimatesCrossBiases) {
                    for (var k = 0; k < COMPONENTS; k++) {
                        value += m(x, mOffset, row, k) * x[gOffset + col * COMPONENTS + k];
                    }
                }
                result[pos++] = value;
            }
        }
        return result;
    }

    private static double m(final double[] x, final int mOffset, final int row, final int col) {
        return x[mOffset + col * COMPONENTS + row];
    }

    private static double[][] numericalJacobian(final double[] params, final int outputs,
                                                final boolean estimatesBias, final boolean estimatesCrossBiases) {
        final var jacobian = new double[outputs][params.length];
        for (var j = 0; j < params.length; j++) {
            final var plus = params.clone();
            final var minus = params.clone();
            plus[j] += STEP;
            minus[j] -= STEP;
            final var fPlus = map(plus, outputs, estimatesBias, estimatesCrossBiases);
            final var fMinus = map(minus, outputs, estimatesBias, estimatesCrossBiases);
            for (var i = 0; i < outputs; i++) {
                jacobian[i][j] = (fPlus[i] - fMinus[i]) / (2.0 * STEP);
            }
        }
        return jacobian;
    }

    private static double[][] propagate(final double[][] jacobian, final Matrix covariance) {
        final var outputs = jacobian.length;
        final var inputs = covariance.getRows();
        final var jc = new double[outputs][inputs];
        for (var i = 0; i < outputs; i++) {
            for (var k = 0; k < inputs; k++) {
                var value = 0.0;
                for (var l = 0; l < inputs; l++) {
                    value += jacobian[i][l] * covariance.getElementAt(l, k);
                }
                jc[i][k] = value;
            }
        }
        final var result = new double[outputs][outputs];
        for (var i = 0; i < outputs; i++) {
            for (var j = 0; j < outputs; j++) {
                var value = 0.0;
                for (var k = 0; k < inputs; k++) {
                    value += jc[i][k] * jacobian[j][k];
                }
                result[i][j] = value;
            }
        }
        return result;
    }

    private static Object getFitter(final Object calibrator) {
        try {
            final var field = calibrator.getClass().getDeclaredField("fitter");
            field.setAccessible(true);
            return field.get(calibrator);
        } catch (final ReflectiveOperationException e) {
            throw new AssertionError("Could not read fitter", e);
        }
    }

    private static double[] getParams(final Object fitter) {
        try {
            return (double[]) fitter.getClass().getMethod("getA").invoke(fitter);
        } catch (final ReflectiveOperationException e) {
            throw new AssertionError("Could not read fitter parameters", e);
        }
    }

    private static Matrix getCovariance(final Object fitter) {
        try {
            return (Matrix) fitter.getClass().getMethod("getCovar").invoke(fitter);
        } catch (final ReflectiveOperationException e) {
            throw new AssertionError("Could not read fitter covariance", e);
        }
    }
}
