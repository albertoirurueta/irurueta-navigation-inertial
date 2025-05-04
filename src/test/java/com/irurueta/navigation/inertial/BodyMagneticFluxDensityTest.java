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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class BodyMagneticFluxDensityTest {

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.

    private static final double MIN_VALUE = 30e-6;
    private static final double MAX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBx(), 0.0);
        assertEquals(0.0, b.getBy(), 0.0);
        assertEquals(0.0, b.getBz(), 0.0);
        assertEquals(0.0, b.getNorm(), 0.0);
        var bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        var by1 = b.getByAsMagneticFluxDensity();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        var bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        var bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        var by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        var bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        var triad1 = b.getCoordinatesAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        var triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        b = new BodyMagneticFluxDensity(bx, by, bz);

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
        final var bNorm = Math.sqrt(bx * bx + by * by + bz * bz);
        assertEquals(bNorm, b.getNorm(), 0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bx, triad1.getValueX(), 0.0);
        assertEquals(by, triad1.getValueY(), 0.0);
        assertEquals(bz, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density measurements
        bx1 = new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
        by1 = new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
        bz1 = new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
        b = new BodyMagneticFluxDensity(bx1, by1, bz1);

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
        assertEquals(bNorm, b.getNorm(), 0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bx, triad1.getValueX(), 0.0);
        assertEquals(by, triad1.getValueY(), 0.0);
        assertEquals(bz, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density triad
        final var triad = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, bx, by, bz);
        b = new BodyMagneticFluxDensity(triad);

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
        assertEquals(bNorm, b.getNorm(), 0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bx, triad1.getValueX(), 0.0);
        assertEquals(by, triad1.getValueY(), 0.0);
        assertEquals(bz, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test copy constructor
        final var b2 = new BodyMagneticFluxDensity(b);

        assertEquals(bx, b2.getBx(), 0.0);
        assertEquals(by, b2.getBy(), 0.0);
        assertEquals(bz, b2.getBz(), 0.0);
        assertEquals(b2.getNorm(), b.getNorm(), 0.0);
        bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(bx, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        by1 = b.getByAsMagneticFluxDensity();
        assertEquals(by, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(bz, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        bx2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        by2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        bz2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bx, triad1.getValueX(), 0.0);
        assertEquals(by, triad1.getValueY(), 0.0);
        assertEquals(bz, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);
    }

    @Test
    void testGetSetBx() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBx(), 0.0);

        // set value
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBx(bx);

        // check
        assertEquals(bx, b.getBx(), 0.0);
    }

    @Test
    void testGetSetBxAsMagneticFluxDensity() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        final var bx1 = b.getBxAsMagneticFluxDensity();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var bx2 = new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
        b.setBx(bx2);

        // check
        final var bx3 = b.getBxAsMagneticFluxDensity();
        final var bx4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBxAsMagneticFluxDensity(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetBy() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBy(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBy(by);

        // check
        assertEquals(by, b.getBy(), 0.0);
    }

    @Test
    void testGetSetByAsMagneticFluxDensity() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        final var by1 = b.getByAsMagneticFluxDensity();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var by2 = new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
        b.setBy(by2);

        // check
        final var by3 = b.getByAsMagneticFluxDensity();
        final var by4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getByAsMagneticFluxDensity(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetBz() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBz(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBz(bz);

        // check
        assertEquals(bz, b.getBz(), 0.0);
    }

    @Test
    void testGetSetBzAsMagneticFluxDensity() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        final var bz1 = b.getBzAsMagneticFluxDensity();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var bz2 = new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
        b.setBz(bz2);

        // check
        final var bz3 = b.getBzAsMagneticFluxDensity();
        final var bz4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBzAsMagneticFluxDensity(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testSetCoordinates1() {
        final var b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBx(), 0.0);
        assertEquals(0.0, b.getBy(), 0.0);
        assertEquals(0.0, b.getBz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
    }

    @Test
    void testSetCoordinates2() {
        final var b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBx(), 0.0);
        assertEquals(0.0, b.getBy(), 0.0);
        assertEquals(0.0, b.getBz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA));

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
    }

    @Test
    void testSetCoordinates3() {
        final var b = new BodyMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBx(), 0.0);
        assertEquals(0.0, b.getBy(), 0.0);
        assertEquals(0.0, b.getBz(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var triad1 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, bx, by, bz);
        b.setCoordinates(triad1);

        // check default values
        assertEquals(bx, b.getBx(), 0.0);
        assertEquals(by, b.getBy(), 0.0);
        assertEquals(bz, b.getBz(), 0.0);
        final var triad2 = b.getCoordinatesAsTriad();
        final var triad3 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad3);
        assertEquals(triad1, triad2);
        assertEquals(triad1, triad3);
    }

    @Test
    void testGetNorm() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getNorm(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check
        assertEquals(Math.sqrt(bx * bx + by * by + bz * bz), b.getNorm(), 0.0);
    }

    @Test
    void testGetNormAsMagneticFluxDensity() {
        final var b = new BodyMagneticFluxDensity();

        // check default value
        final var norm1 = b.getNormAsMagneticFluxDensity();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bx, by, bz);

        // check
        final var norm = Math.sqrt(bx * bx + by * by + bz * bz);
        final var norm2 = b.getNormAsMagneticFluxDensity();
        assertEquals(norm, norm2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm2.getUnit());
        final var norm3 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getNormAsMagneticFluxDensity(norm3);
        assertEquals(norm2, norm3);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b2 = new BodyMagneticFluxDensity();

        b1.copyTo(b2);

        // check
        assertEquals(b1.getBx(), b2.getBx(), 0.0);
        assertEquals(b1.getBy(), b2.getBy(), 0.0);
        assertEquals(b1.getBz(), b2.getBz(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b2 = new BodyMagneticFluxDensity();

        b2.copyFrom(b1);

        // check
        assertEquals(b1.getBx(), b2.getBx(), 0.0);
        assertEquals(b1.getBy(), b2.getBy(), 0.0);
        assertEquals(b1.getBz(), b2.getBz(), 0.0);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b = new BodyMagneticFluxDensity(bx, by, bz);

        final var array1 = b.asArray();
        final var array2 = new double[BodyMagneticFluxDensity.COMPONENTS];
        b.asArray(array2);

        // check
        assertEquals(bx, array1[0], 0.0);
        assertEquals(by, array1[1], 0.0);
        assertEquals(bz, array1[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> b.asArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b = new BodyMagneticFluxDensity(bx, by, bz);

        final var m1 = b.asMatrix();
        final var m2 = new Matrix(1, 1);
        b.asMatrix(m2);
        final var m3 = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        b.asMatrix(m3);

        // check
        assertEquals(bx, m1.getElementAtIndex(0), 0.0);
        assertEquals(by, m1.getElementAtIndex(1), 0.0);
        assertEquals(bz, m1.getElementAtIndex(2), 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b2 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b3 = new BodyMagneticFluxDensity();

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b2 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b3 = new BodyMagneticFluxDensity();

        //noinspection EqualsWithItself
        assertEquals(b1, b1);
        //noinspection EqualsWithItself
        assertTrue(b1.equals(b1));
        assertTrue(b1.equals(b2));
        assertFalse(b1.equals(b3));
        assertNotEquals(null, b1);
        assertFalse(b1.equals(null));
        assertNotEquals(new Object(), b1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b2 = new BodyMagneticFluxDensity(bx, by, bz);
        final var b3 = new BodyMagneticFluxDensity();

        assertTrue(b1.equals(b1, THRESHOLD));
        assertTrue(b1.equals(b2, THRESHOLD));
        assertFalse(b1.equals(b3, THRESHOLD));
        assertFalse(b1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);

        final var b2 = b1.clone();

        // check
        assertEquals(b1, b2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var by = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bz = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new BodyMagneticFluxDensity(bx, by, bz);

        final var bytes = SerializationHelper.serialize(b1);

        final var b2 = SerializationHelper.deserialize(bytes);

        assertEquals(b1, b2);
        assertNotSame(b1, b2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = BodyMagneticFluxDensity.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
