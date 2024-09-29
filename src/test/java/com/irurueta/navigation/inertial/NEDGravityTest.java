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
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class NEDGravityTest {

    private static final double MIN_VALUE = 9.80;
    private static final double MAX_VALUE = 9.82;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        NEDGravity gravity = new NEDGravity();

        // check default values
        assertEquals(0.0, gravity.getGn(), 0.0);
        assertEquals(0.0, gravity.getGe(), 0.0);
        assertEquals(0.0, gravity.getGd(), 0.0);
        assertEquals(0.0, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getNorm(), 0.0);
        assertEquals(0.0, gravity.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor with gravity coordinates
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gn * gn + gd * gd);

        gravity = new NEDGravity(gn, gd);

        // check default values
        assertEquals(gn, gravity.getGn(), 0.0);
        assertEquals(0.0, gravity.getGe(), 0.0);
        assertEquals(gd, gravity.getGd(), 0.0);
        assertEquals(gn, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gd, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravity.getNorm(), 0.0);
        assertEquals(g, gravity.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor with acceleration coordinates
        final Acceleration gravityN = new Acceleration(gn, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityD = new Acceleration(gd, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity = new NEDGravity(gravityN, gravityD);

        // check default values
        assertEquals(gn, gravity.getGn(), 0.0);
        assertEquals(0.0, gravity.getGe(), 0.0);
        assertEquals(gd, gravity.getGd(), 0.0);
        assertEquals(gn, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gd, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravity.getNorm(), 0.0);
        assertEquals(g, gravity.getNormAsAcceleration().getValue().doubleValue(), 0.0);

        // test constructor from another gravity
        final NEDGravity gravity2 = new NEDGravity(gravity);

        // check default values
        assertEquals(gn, gravity2.getGn(), 0.0);
        assertEquals(0.0, gravity2.getGe(), 0.0);
        assertEquals(gd, gravity2.getGd(), 0.0);
        assertEquals(gn, gravity2.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity2.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gd, gravity2.getGdAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(g, gravity2.getNorm(), 0.0);
        assertEquals(g, gravity2.getNormAsAcceleration().getValue().doubleValue(), 0.0);
    }

    @Test
    public void testGetSetGn() {
        final NEDGravity gravity = new NEDGravity();

        // check default value
        assertEquals(0.0, gravity.getGn(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setGn(gn);

        // check
        assertEquals(gn, gravity.getGn(), 0.0);
    }

    @Test
    public void testGetSetGd() {
        final NEDGravity gravity = new NEDGravity();

        // check default value
        assertEquals(0.0, gravity.getGd(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setGd(gd);

        // check
        assertEquals(gd, gravity.getGd(), 0.0);
    }

    @Test
    public void testSetCoordinates() {
        final NEDGravity gravity = new NEDGravity();

        // check default values
        assertEquals(0.0, gravity.getGn(), 0.0);
        assertEquals(0.0, gravity.getGe(), 0.0);
        assertEquals(0.0, gravity.getGd(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        gravity.setCoordinates(gn, gd);

        // check
        assertEquals(gn, gravity.getGn(), 0.0);
        assertEquals(0.0, gravity.getGe(), 0.0);
        assertEquals(gd, gravity.getGd(), 0.0);
    }

    @Test
    public void testGetSetGnAsAcceleration() {
        final NEDGravity gravity = new NEDGravity();

        // check default value
        assertEquals(0.0, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityN1 = new Acceleration(gn, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGn(gravityN1);

        // check
        assertEquals(gn, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);

        final Acceleration gravityN2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGnAsAcceleration(gravityN2);
        final Acceleration gravityN3 = gravity.getGnAsAcceleration();
        assertEquals(gn, gravityN2.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravityN2.getUnit());
        assertEquals(gravityN1, gravityN2);
        assertEquals(gravityN1, gravityN3);
    }

    @Test
    public void testGetGeAsAcceleration() {
        final NEDGravity gravity = new NEDGravity();

        // check
        final Acceleration gravityE1 = new Acceleration(MIN_VALUE, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravity.getGeAsAcceleration(gravityE1);
        final Acceleration gravityE2 = gravity.getGeAsAcceleration();
        assertEquals(0.0, gravityE1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravityE1.getUnit());
        assertEquals(gravityE1, gravityE2);
    }

    @Test
    public void testGetSetGdAsAcceleration() {
        final NEDGravity gravity = new NEDGravity();

        // check default value
        assertEquals(0.0, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityD1 = new Acceleration(gd, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setGd(gravityD1);

        // check
        assertEquals(gd, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);

        final Acceleration gravityD2 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        gravity.getGdAsAcceleration(gravityD2);
        final Acceleration gravityD3 = gravity.getGdAsAcceleration();
        assertEquals(gd, gravityD2.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, gravityD2.getUnit());
        assertEquals(gravityD1, gravityD2);
        assertEquals(gravityD1, gravityD3);
    }

    @Test
    public void testSetCoordinatesFromAccelerations() {
        final NEDGravity gravity = new NEDGravity();

        // check default values
        assertEquals(0.0, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration gravityN = new Acceleration(gn, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration gravityD = new Acceleration(gd, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        gravity.setCoordinates(gravityN, gravityD);

        // check
        assertEquals(gn, gravity.getGnAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(0.0, gravity.getGeAsAcceleration().getValue().doubleValue(), 0.0);
        assertEquals(gd, gravity.getGdAsAcceleration().getValue().doubleValue(), 0.0);
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double g = Math.sqrt(gn * gn + gd * gd);

        final NEDGravity gravity = new NEDGravity(gn, gd);

        assertEquals(g, gravity.getNorm(), 0.0);

        final Acceleration norm1 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        gravity.getNormAsAcceleration(norm1);
        final Acceleration norm2 = gravity.getNormAsAcceleration();

        assertEquals(g, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        assertEquals(norm1, norm2);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);
        final NEDGravity gravity2 = new NEDGravity();

        gravity1.copyTo(gravity2);

        // check
        assertEquals(gravity1.getGn(), gravity2.getGn(), 0.0);
        assertEquals(gravity1.getGe(), gravity2.getGe(), 0.0);
        assertEquals(gravity1.getGd(), gravity2.getGd(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);
        final NEDGravity gravity2 = new NEDGravity();

        gravity2.copyFrom(gravity1);

        // check
        assertEquals(gravity1.getGn(), gravity2.getGn(), 0.0);
        assertEquals(gravity1.getGe(), gravity2.getGe(), 0.0);
        assertEquals(gravity1.getGd(), gravity2.getGd(), 0.0);
    }

    @Test
    public void testAsArray() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity = new NEDGravity(gn, gd);

        final double[] array1 = gravity.asArray();
        final double[] array2 = new double[NEDGravity.COMPONENTS];
        gravity.asArray(array2);

        // check
        assertEquals(gn, array1[0], 0.0);
        assertEquals(0.0, array1[1], 0.0);
        assertEquals(gd, array1[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> gravity.asArray(new double[1]));
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity = new NEDGravity(gn, gd);

        final Matrix matrix1 = gravity.asMatrix();
        final Matrix matrix2 = new Matrix(1, 1);
        gravity.asMatrix(matrix2);
        final Matrix matrix3 = new Matrix(NEDGravity.COMPONENTS, 1);
        gravity.asMatrix(matrix3);

        // check
        assertEquals(gn, matrix1.getElementAtIndex(0), 0.0);
        assertEquals(0.0, matrix1.getElementAtIndex(1), 0.0);
        assertEquals(gd, matrix1.getElementAtIndex(2), 0.0);
        assertEquals(matrix1, matrix2);
        assertEquals(matrix1, matrix3);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);
        final NEDGravity gravity2 = new NEDGravity(gn, gd);
        final NEDGravity gravity3 = new NEDGravity();

        assertEquals(gravity1.hashCode(), gravity2.hashCode());
        assertNotEquals(gravity1.hashCode(), gravity3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);
        final NEDGravity gravity2 = new NEDGravity(gn, gd);
        final NEDGravity gravity3 = new NEDGravity();

        //noinspection EqualsWithItself
        assertEquals(gravity1, gravity1);
        //noinspection EqualsWithItself
        assertTrue(gravity1.equals(gravity1));
        assertTrue(gravity1.equals(gravity2));
        assertFalse(gravity1.equals(gravity3));
        assertNotEquals(gravity1, null);
        assertFalse(gravity1.equals(null));
        assertNotEquals(gravity1, new Object());
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);
        final NEDGravity gravity2 = new NEDGravity(gn, gd);
        final NEDGravity gravity3 = new NEDGravity();

        assertTrue(gravity1.equals(gravity1, THRESHOLD));
        assertTrue(gravity1.equals(gravity2, THRESHOLD));
        assertFalse(gravity1.equals(gravity3, THRESHOLD));
        assertFalse(gravity1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);

        final Object gravity2 = gravity1.clone();

        // check
        assertEquals(gravity1, gravity2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final NEDGravity gravity1 = new NEDGravity(gn, gd);

        final byte[] bytes = SerializationHelper.serialize(gravity1);

        final NEDGravity gravity2 = SerializationHelper.deserialize(bytes);

        assertEquals(gravity1, gravity2);
        assertNotSame(gravity1, gravity2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = NEDGravity.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
