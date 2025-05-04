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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;

class BodyMagneticFluxDensityGeneratorTest {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_MAGNETIC_FLUX = -70e-6;
    private static final double MAX_MAGNETIC_FLUX = 70e-6;

    private static final int N = 100;

    @Test
    void testGenerate1() {
        final var randomizer = new UniformRandomizer();

        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIron();

        final var trueMagneticFluxDensities = new ArrayList<BodyMagneticFluxDensity>();
        final var expectedMagneticFluxDensities = new ArrayList<BodyMagneticFluxDensity>();
        for (var i = 0; i < N; i++) {
            final var trueMagneticFluxDensity = generateTruth(randomizer);
            final var expectedMagneticFluxDensity = generateExpected(trueMagneticFluxDensity, hardIron, softIron);

            trueMagneticFluxDensities.add(trueMagneticFluxDensity);
            expectedMagneticFluxDensities.add(expectedMagneticFluxDensity);
        }

        final var result = BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensities, hardIron, softIron);

        assertEquals(expectedMagneticFluxDensities, result);
    }

    @Test
    void testGenerate2() {
        final var randomizer = new UniformRandomizer();

        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIron();

        final var trueMagneticFluxDensities = new ArrayList<BodyMagneticFluxDensity>();
        final var expectedMagneticFluxDensities = new ArrayList<BodyMagneticFluxDensity>();
        for (var i = 0; i < N; i++) {
            final var trueMagneticFluxDensity = generateTruth(randomizer);
            final var expectedMagneticFluxDensity = generateExpected(trueMagneticFluxDensity, hardIron, softIron);

            trueMagneticFluxDensities.add(trueMagneticFluxDensity);
            expectedMagneticFluxDensities.add(expectedMagneticFluxDensity);
        }

        final var result = new ArrayList<BodyMagneticFluxDensity>();
        BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensities, hardIron, softIron, result);

        assertEquals(expectedMagneticFluxDensities, result);
    }

    @Test
    void testGenerate3() {
        final var randomizer = new UniformRandomizer();

        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIron();

        final var trueMagneticFluxDensity = generateTruth(randomizer);
        final var expectedMagneticFluxDensity = generateExpected(trueMagneticFluxDensity, hardIron, softIron);

        final var result = BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensity, hardIron, softIron);

        assertEquals(expectedMagneticFluxDensity, result);
    }

    @Test
    void testGenerate4() {
        final var randomizer = new UniformRandomizer();

        final var hardIron = generateHardIron(randomizer);
        final var softIron = generateSoftIron();

        final var trueMagneticFluxDensity = generateTruth(randomizer);
        final var expectedMagneticFluxDensity = generateExpected(trueMagneticFluxDensity, hardIron, softIron);

        final var result = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityGenerator.generate(trueMagneticFluxDensity, hardIron, softIron, result);

        assertEquals(expectedMagneticFluxDensity, result);
    }

    private static BodyMagneticFluxDensity generateTruth(final UniformRandomizer randomizer) {
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX, MAX_MAGNETIC_FLUX);
        return new BodyMagneticFluxDensity(bx, by, bz);
    }

    private static BodyMagneticFluxDensity generateExpected(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        try {
            final var bTrue = input.asMatrix();
            final var tmp = Matrix.identity(BodyMagneticFluxDensity.COMPONENTS, BodyMagneticFluxDensity.COMPONENTS);
            tmp.add(softIron);
            tmp.multiply(bTrue);

            tmp.setElementAtIndex(0, tmp.getElementAtIndex(0) + hardIron[0]);
            tmp.setElementAtIndex(1, tmp.getElementAtIndex(1) + hardIron[1]);
            tmp.setElementAtIndex(2, tmp.getElementAtIndex(2) + hardIron[2]);

            return new BodyMagneticFluxDensity(
                    tmp.getElementAtIndex(0), tmp.getElementAtIndex(1), tmp.getElementAtIndex(2));
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIron() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }
}
