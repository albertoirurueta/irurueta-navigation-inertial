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
 * covariance of the scale-factor and cross-coupling unknowns unchanged.
 * <p>
 * The fitter stores matrix M = Mg + I column-wise (m11, m21, m31, m12, m22, m32,
 * m13, m23, m33). The published parameters sx, sy, sz, mxy, mxz, myx, myz, mzx
 * and mzy are each equal to one entry of M, up to a constant offset. Their
 * Jacobian rows therefore contain a single 1.0, and their propagated covariance
 * block must equal the matching block of the fitter covariance.
 */
final class GyroscopeCovariancePropagationAssert {

    /**
     * Number of published scale-factor and cross-coupling parameters.
     */
    private static final int SCALE_AND_CROSS_COUPLING_PARAMS = 9;

    /**
     * Position of sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy within the column-wise
     * M block of fitter unknowns.
     */
    private static final int[] M_INDICES = {0, 4, 8, 3, 6, 1, 7, 2, 5};

    /**
     * Relative tolerance. Each Jacobian row has a single unit entry, so values
     * must match to rounding error.
     */
    private static final double RELATIVE_ERROR = 1e-12;

    private GyroscopeCovariancePropagationAssert() {
    }

    /**
     * Asserts that the propagated covariance of sx..mzy equals the fitter
     * covariance of the matching M entries.
     *
     * @param calibrator          calibrator that has finished a general-case calibration.
     * @param estimatedCovariance covariance returned by the calibrator.
     * @param outputOffset        index of sx in the published covariance (3 when biases
     *                            are estimated, 0 when they are known).
     * @param fitterOffset        index of m11 in the fitter unknowns (3 when biases are
     *                            estimated, 0 when they are known).
     */
    static void assertScaleAndCrossCouplingCovariancePropagated(
            final Object calibrator, final Matrix estimatedCovariance, final int outputOffset,
            final int fitterOffset) {
        final var fitterCovariance = getFitterCovariance(calibrator);

        for (var i = 0; i < SCALE_AND_CROSS_COUPLING_PARAMS; i++) {
            for (var j = 0; j < SCALE_AND_CROSS_COUPLING_PARAMS; j++) {
                final var expected = fitterCovariance.getElementAt(fitterOffset + M_INDICES[i],
                        fitterOffset + M_INDICES[j]);
                final var actual = estimatedCovariance.getElementAt(outputOffset + i, outputOffset + j);
                assertEquals(expected, actual, RELATIVE_ERROR * Math.abs(expected),
                        "covariance entry (" + (outputOffset + i) + ", " + (outputOffset + j) + ")");
            }
        }

        // mzy is the last entry; its variance must be non-trivial so that a
        // scaled Jacobian entry cannot go unnoticed
        final var mzyIndex = outputOffset + SCALE_AND_CROSS_COUPLING_PARAMS - 1;
        assertTrue(estimatedCovariance.getElementAt(mzyIndex, mzyIndex) > 0.0);
    }

    private static Matrix getFitterCovariance(final Object calibrator) {
        try {
            final var field = calibrator.getClass().getDeclaredField("fitter");
            field.setAccessible(true);
            final var fitter = field.get(calibrator);
            return (Matrix) fitter.getClass().getMethod("getCovar").invoke(fitter);
        } catch (final ReflectiveOperationException e) {
            throw new AssertionError("Could not read fitter covariance", e);
        }
    }
}
