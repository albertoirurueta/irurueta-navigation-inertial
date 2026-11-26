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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import org.junit.jupiter.api.Test;

import java.util.Objects;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Tests the {@code hashCode()} contract defined by {@link GravityOrGravitation} and its consistency with the
 * {@code equals(Object)} implementations provided by the concrete subclasses {@link ECEFGravity} and
 * {@link ECIGravitation}.
 */
class GravityOrGravitationTest {

    /**
     * Acceleration due to gravity through x-axis used in tests, expressed in meters per squared second (m/s^2).
     */
    private static final double GX = 9.801;

    /**
     * Acceleration due to gravity through y-axis used in tests, expressed in meters per squared second (m/s^2).
     */
    private static final double GY = 9.805;

    /**
     * Acceleration due to gravity through z-axis used in tests, expressed in meters per squared second (m/s^2).
     */
    private static final double GZ = 9.819;

    /**
     * Acceleration through x-axis that differs from {@link #GX}, expressed in meters per squared second (m/s^2).
     */
    private static final double OTHER_GX = 9.811;

    /**
     * Acceleration through y-axis that differs from {@link #GY}, expressed in meters per squared second (m/s^2).
     */
    private static final double OTHER_GY = 9.815;

    /**
     * Acceleration through z-axis that differs from {@link #GZ}, expressed in meters per squared second (m/s^2).
     */
    private static final double OTHER_GZ = 9.829;

    /**
     * Threshold used to compare instances, smaller than the difference between test components.
     */
    private static final double THRESHOLD = 1e-6;

    /**
     * Number of columns of a matrix that does not have the expected column vector size.
     */
    private static final int WRONG_COLUMNS = 2;

    /**
     * Checks that two ECEF gravity instances with equal components are equal and have equal hash codes.
     */
    @Test
    void hashCode_whenEcefGravityComponentsAreEqual_returnsEqualHashCodes() {
        final var gravity1 = new ECEFGravity(GX, GY, GZ);
        final var gravity2 = new ECEFGravity(GX, GY, GZ);

        assertEquals(gravity1, gravity2);
        assertEquals(gravity1.hashCode(), gravity2.hashCode());
    }

    /**
     * Checks that two ECI gravitation instances with equal components are equal and have equal hash codes.
     */
    @Test
    void hashCode_whenEciGravitationComponentsAreEqual_returnsEqualHashCodes() {
        final var gravitation1 = new ECIGravitation(GX, GY, GZ);
        final var gravitation2 = new ECIGravitation(GX, GY, GZ);

        assertEquals(gravitation1, gravitation2);
        assertEquals(gravitation1.hashCode(), gravitation2.hashCode());
    }

    /**
     * Checks that the hash code is derived only from the gravity components, as defined by the base class.
     */
    @Test
    void hashCode_whenComponentsAreSet_returnsHashOfComponents() {
        final var expected = Objects.hash(GX, GY, GZ);

        assertEquals(expected, new ECEFGravity(GX, GY, GZ).hashCode());
        assertEquals(expected, new ECIGravitation(GX, GY, GZ).hashCode());
    }

    /**
     * Checks that instances whose components differ are not equal and have different hash codes.
     */
    @Test
    void hashCode_whenComponentsDiffer_returnsDifferentHashCodesAndNotEqual() {
        final var gravity1 = new ECEFGravity(GX, GY, GZ);
        final var gravity2 = new ECEFGravity(OTHER_GX, GY, GZ);
        final var gravitation1 = new ECIGravitation(GX, GY, GZ);
        final var gravitation2 = new ECIGravitation(OTHER_GX, GY, GZ);

        assertNotEquals(gravity1, gravity2);
        assertNotEquals(gravity1.hashCode(), gravity2.hashCode());
        assertNotEquals(gravitation1, gravitation2);
        assertNotEquals(gravitation1.hashCode(), gravitation2.hashCode());
    }

    /**
     * Checks that an ECEF gravity and an ECI gravitation with equal components are not equal to each other
     * while still sharing a hash code, which is allowed by the {@code equals}/{@code hashCode} contract.
     */
    @Test
    void equals_whenEcefGravityAndEciGravitationHaveEqualComponents_returnsFalseWithEqualHashCodes() {
        final var gravity = new ECEFGravity(GX, GY, GZ);
        final var gravitation = new ECIGravitation(GX, GY, GZ);

        assertNotEquals(gravity, gravitation);
        assertNotEquals(gravitation, gravity);
        assertEquals(gravity.hashCode(), gravitation.hashCode());
    }

    /**
     * Checks that copies made through the copy constructors are equal to their source and have the same hash code.
     */
    @Test
    void equals_whenInstancesAreCopied_returnsTrueWithEqualHashCodes() {
        final var gravity = new ECEFGravity(GX, GY, GZ);
        final var gravityCopy = new ECEFGravity(gravity);
        final var gravitation = new ECIGravitation(GX, GY, GZ);
        final var gravitationCopy = new ECIGravitation(gravitation);

        assertTrue(gravity.equals(gravityCopy) && gravityCopy.equals(gravity));
        assertEquals(gravity.hashCode(), gravityCopy.hashCode());
        assertTrue(gravitation.equals(gravitationCopy) && gravitationCopy.equals(gravitation));
        assertEquals(gravitation.hashCode(), gravitationCopy.hashCode());
    }

    /**
     * Checks that threshold comparison returns false when only the y or only the z component differs.
     */
    @Test
    void equalsWithThreshold_whenOnlyYOrZComponentDiffers_returnsFalse() {
        final var gravity = new ECEFGravity(GX, GY, GZ);

        assertTrue(gravity.equals(new ECEFGravity(GX, GY, GZ), THRESHOLD));
        assertFalse(gravity.equals(new ECEFGravity(GX, OTHER_GY, GZ), THRESHOLD));
        assertFalse(gravity.equals(new ECEFGravity(GX, GY, OTHER_GZ), THRESHOLD));

        final var gravitation = new ECIGravitation(GX, GY, GZ);

        assertFalse(gravitation.equals(new ECIGravitation(GX, OTHER_GY, GZ), THRESHOLD));
        assertFalse(gravitation.equals(new ECIGravitation(GX, GY, OTHER_GZ), THRESHOLD));
    }

    /**
     * Checks that a matrix with the expected number of rows but a wrong number of columns is resized.
     *
     * @throws WrongSizeException never thrown.
     */
    @Test
    void asMatrix_whenMatrixHasWrongColumns_resizesAndFillsComponents() throws WrongSizeException {
        final var gravity = new ECEFGravity(GX, GY, GZ);
        final var result = new Matrix(GravityOrGravitation.COMPONENTS, WRONG_COLUMNS);

        gravity.asMatrix(result);

        assertEquals(GravityOrGravitation.COMPONENTS, result.getRows());
        assertEquals(1, result.getColumns());
        assertEquals(GX, result.getElementAtIndex(0), 0.0);
        assertEquals(GY, result.getElementAtIndex(1), 0.0);
        assertEquals(GZ, result.getElementAtIndex(2), 0.0);
    }
}
