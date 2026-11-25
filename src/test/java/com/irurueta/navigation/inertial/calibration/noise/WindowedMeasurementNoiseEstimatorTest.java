/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.navigation.LockedException;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests the behavior of {@link WindowedMeasurementNoiseEstimator} when the window is not completely filled.
 * A concrete implementation of the estimator is used to exercise the base class logic.
 */
class WindowedMeasurementNoiseEstimatorTest {

    /**
     * Window size used in tests.
     */
    private static final int WINDOW_SIZE = 5;

    /**
     * First sample value.
     */
    private static final double VALUE1 = 1.0;

    /**
     * Second sample value.
     */
    private static final double VALUE2 = 2.0;

    /**
     * Third sample value.
     */
    private static final double VALUE3 = 6.0;

    /**
     * Maximum allowed error.
     */
    private static final double ABSOLUTE_ERROR = 1e-12;

    /**
     * Checks that average and variance are computed over the stored samples when the window is not full.
     *
     * @throws LockedException if estimator is locked (never expected).
     */
    @Test
    void addMeasurementAndProcess_whenWindowNotFilled_computesStatsOverStoredSamples() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();
        estimator.setWindowSize(WINDOW_SIZE);

        estimator.addMeasurementAndProcess(VALUE1);
        estimator.addMeasurementAndProcess(VALUE2);
        estimator.addMeasurementAndProcess(VALUE3);

        assertFalse(estimator.isWindowFilled());
        assertEquals(3, estimator.getNumberOfSamplesInWindow());

        final var avg = (VALUE1 + VALUE2 + VALUE3) / 3.0;
        final var variance = (square(VALUE1 - avg) + square(VALUE2 - avg) + square(VALUE3 - avg)) / 2.0;

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);
        assertEquals(variance, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(Math.sqrt(variance), estimator.getStandardDeviation(), ABSOLUTE_ERROR);
    }

    /**
     * Checks that a single sample yields its own value as average and zero variance.
     *
     * @throws LockedException if estimator is locked (never expected).
     */
    @Test
    void addMeasurementAndProcess_whenSingleSample_returnsZeroVariance() throws LockedException {
        final var estimator = new WindowedAccelerationMeasurementNoiseEstimator();
        estimator.setWindowSize(WINDOW_SIZE);

        estimator.addMeasurementAndProcess(VALUE3);

        assertEquals(VALUE3, estimator.getAvg(), ABSOLUTE_ERROR);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertTrue(Double.isFinite(estimator.getVariance()));
    }

    /**
     * Computes the square of a value.
     *
     * @param value value to be squared.
     * @return squared value.
     */
    private static double square(final double value) {
        return value * value;
    }
}
