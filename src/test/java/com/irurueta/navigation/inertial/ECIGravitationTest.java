/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class ECIGravitationTest {

    private static final double MIN_VALUE = 9.80;
    private static final double MAX_VALUE = 9.82;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var gravitation = new ECIGravitation();

        // check default values
        assertEquals(0.0, gravitation.getGx(), 0.0);
        assertEquals(0.0, gravitation.getGy(), 0.0);
        assertEquals(0.0, gravitation.getGz(), 0.0);
        assertEquals(0.0, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravitation.getNorm(), 0.0);
        assertEquals(0.0, gravitation.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor with gravitation coordinates
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        gravitation = new ECIGravitation(gx, gy, gz);

        // check default values
        assertEquals(gx, gravitation.getGx(), 0.0);
        assertEquals(gy, gravitation.getGy(), 0.0);
        assertEquals(gz, gravitation.getGz(), 0.0);
        assertEquals(gx, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gy, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gz, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravitation.getNorm(), 0.0);
        assertEquals(g, gravitation.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor with acceleration coordinates
        final var gravitationX = new Acceleration(gx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var gravitationY = new Acceleration(gy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var gravitationZ = new Acceleration(gz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation = new ECIGravitation(gravitationX, gravitationY, gravitationZ);

        // check default values
        assertEquals(gx, gravitation.getGx(), 0.0);
        assertEquals(gy, gravitation.getGy(), 0.0);
        assertEquals(gz, gravitation.getGz(), 0.0);
        assertEquals(gx, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gy, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gz, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravitation.getNorm(), 0.0);
        assertEquals(g, gravitation.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor from another gravitation
        final var gravitation2 = new ECIGravitation(gravitation);

        // check default values
        assertEquals(gx, gravitation2.getGx(), 0.0);
        assertEquals(gy, gravitation2.getGy(), 0.0);
        assertEquals(gz, gravitation2.getGz(), 0.0);
        assertEquals(gx, gravitation2.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gy, gravitation2.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gz, gravitation2.getGzAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravitation2.getNorm(), 0.0);
        assertEquals(g, gravitation2.getNormAsAcceleration().getValue().doubleValue(), 0.0);
    }

    @Test
    void testGetSetGx() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGx(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGx(gx);

        // check
        assertEquals(gx, gravitation.getGx(), 0.0);
    }

    @Test
    void testGetSetGy() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGy(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGy(gy);

        // check
        assertEquals(gy, gravitation.getGy(), 0.0);
    }

    @Test
    void testGetSetGz() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGz(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setGz(gz);

        // check
        assertEquals(gz, gravitation.getGz(), 0.0);
    }

    @Test
    void testSetCoordinates() {
        final var gravitation = new ECIGravitation();

        // check default values
        assertEquals(0.0, gravitation.getGx(), 0.0);
        assertEquals(0.0, gravitation.getGy(), 0.0);
        assertEquals(0.0, gravitation.getGz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravitation.setCoordinates(gx, gy, gz);

        // check
        assertEquals(gx, gravitation.getGx(), 0.0);
        assertEquals(gy, gravitation.getGy(), 0.0);
        assertEquals(gz, gravitation.getGz(), 0.0);
    }

    @Test
    void testGetSetGxAsAcceleration() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gravitationX1 = new Acceleration(gx, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGx(gravitationX1);

        // check
        assertEquals(gx, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);

        final var gravitationX2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGxAsAcceleration(gravitationX2);
        final var gravitationX3 = gravitation.getGxAsAcceleration();
        assertEquals(gx, gravitationX2.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravitationX2.getUnit());
        assertEquals(gravitationX1, gravitationX2);
        assertEquals(gravitationX1, gravitationX3);
    }

    @Test
    void testGetSetGyAsAcceleration() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gravitationY1 = new Acceleration(gy, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGy(gravitationY1);

        // check
        assertEquals(gy, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);

        final var gravitationY2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGyAsAcceleration(gravitationY2);
        final var gravitationY3 = gravitation.getGyAsAcceleration();
        assertEquals(gy, gravitationY2.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravitationY2.getUnit());
        assertEquals(gravitationY1, gravitationY2);
        assertEquals(gravitationY1, gravitationY3);
    }

    @Test
    void testGetSetGzAsAcceleration() {
        final var gravitation = new ECIGravitation();

        // check default value
        assertEquals(0.0, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gravitationZ1 = new Acceleration(gz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setGz(gravitationZ1);

        // check
        assertEquals(gz, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);

        final var gravitationZ2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravitation.getGzAsAcceleration(gravitationZ2);
        final var gravitationZ3 = gravitation.getGzAsAcceleration();
        assertEquals(gz, gravitationZ2.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravitationZ2.getUnit());
        assertEquals(gravitationZ1, gravitationZ2);
        assertEquals(gravitationZ1, gravitationZ3);
    }

    @Test
    void testSetCoordinatesFromAccelerations() {
        final var gravitation = new ECIGravitation();

        // check default values
        assertEquals(0.0, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gravitationX1 = new Acceleration(gx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var gravitationY1 = new Acceleration(gy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var gravitationZ1 = new Acceleration(gz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravitation.setCoordinates(gravitationX1, gravitationY1, gravitationZ1);

        // check
        assertEquals(gx, gravitation.getGxAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gy, gravitation.getGyAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gz, gravitation.getGzAsAcceleration().getValue().doubleValue(), 0.0);
    }

    @Test
    void testGetNorm() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var g = Math.sqrt(gx * gx + gy * gy + gz * gz);

        final var gravitation = new ECIGravitation(gx, gy, gz);

        assertEquals(g, gravitation.getNorm(), 0.0);

        final var norm1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravitation.getNormAsAcceleration(norm1);
        final var norm2 = gravitation.getNormAsAcceleration();

        assertEquals(g, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        assertEquals(norm1, norm2);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);
        final var gravitation2 = new ECIGravitation();

        gravitation1.copyTo(gravitation2);

        // check
        assertEquals(gravitation1.getGx(), gravitation2.getGx(), 0.0);
        assertEquals(gravitation1.getGy(), gravitation2.getGy(), 0.0);
        assertEquals(gravitation1.getGz(), gravitation2.getGz(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);
        final var gravitation2 = new ECIGravitation();

        gravitation2.copyFrom(gravitation1);

        // check
        assertEquals(gravitation1.getGx(), gravitation2.getGx(), 0.0);
        assertEquals(gravitation1.getGy(), gravitation2.getGy(), 0.0);
        assertEquals(gravitation1.getGz(), gravitation2.getGz(), 0.0);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation = new ECIGravitation(gx, gy, gz);

        final var array1 = gravitation.asArray();
        final var array2 = new double[ECIGravitation.COMPONENTS];
        gravitation.asArray(array2);

        // check
        assertEquals(gx, array1[0], 0.0);
        assertEquals(gy, array1[1], 0.0);
        assertEquals(gz, array2[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> gravitation.asArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation = new ECIGravitation(gx, gy, gz);

        final var matrix1 = gravitation.asMatrix();
        final var matrix2 = new Matrix(1, 1);
        gravitation.asMatrix(matrix2);
        final var matrix3 = new Matrix(ECIGravitation.COMPONENTS, 1);
        gravitation.asMatrix(matrix3);

        // check
        assertEquals(gx, matrix1.getElementAtIndex(0), 0.0);
        assertEquals(gy, matrix1.getElementAtIndex(1), 0.0);
        assertEquals(gz, matrix1.getElementAtIndex(2), 0.0);
        assertEquals(matrix1, matrix2);
        assertEquals(matrix1, matrix3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);
        final var gravitation2 = new ECIGravitation(gx, gy, gz);
        final var gravitation3 = new ECIGravitation();

        assertEquals(gravitation1.hashCode(), gravitation2.hashCode());
        assertNotEquals(gravitation1.hashCode(), gravitation3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);
        final var gravitation2 = new ECIGravitation(gx, gy, gz);
        final var gravitation3 = new ECIGravitation();

        //noinspection EqualsWithItself
        assertEquals(gravitation1, gravitation1);
        //noinspection EqualsWithItself
        assertTrue(gravitation1.equals(gravitation1));
        assertTrue(gravitation1.equals(gravitation2));
        assertFalse(gravitation1.equals(gravitation3));
        assertNotEquals(null, gravitation1);
        assertFalse(gravitation1.equals(null));
        assertNotEquals(new Object(), gravitation1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);
        final var gravitation2 = new ECIGravitation(gx, gy, gz);
        final var gravitation3 = new ECIGravitation();

        assertTrue(gravitation1.equals(gravitation1, THRESHOLD));
        assertTrue(gravitation1.equals(gravitation2, THRESHOLD));
        assertFalse(gravitation1.equals(gravitation3, THRESHOLD));
        assertFalse(gravitation1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);

        final var gravitation2 = gravitation1.clone();

        // check
        assertEquals(gravitation1, gravitation2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var gx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gy = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var gravitation1 = new ECIGravitation(gx, gy, gz);

        final var bytes = SerializationHelper.serialize(gravitation1);

        final var gravitation2 = SerializationHelper.deserialize(bytes);

        assertEquals(gravitation1, gravitation2);
        assertNotSame(gravitation1, gravitation2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = ECIGravitation.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
