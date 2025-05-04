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
package com.irurueta.navigation.inertial.wmm;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class NEDMagneticFluxDensityTest {

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.

    private static final double MIN_VALUE = 30e-6;
    private static final double MAX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBn(), 0.0);
        assertEquals(0.0, b.getBe(), 0.0);
        assertEquals(0.0, b.getBd(), 0.0);
        assertEquals(0.0, b.getNorm(), 0.0);
        var bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(0.0, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());
        var be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(0.0, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());
        var bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(0.0, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());
        var bn2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        var be2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        var bd2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
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
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        b = new NEDMagneticFluxDensity(bn, be, bd);

        // check default values
        assertEquals(b.getBn(), bn, 0.0);
        assertEquals(b.getBe(), be, 0.0);
        assertEquals(b.getBd(), bd, 0.0);
        final var bNorm = Math.sqrt(bn * bn + be * be + bd * bd);
        assertEquals(b.getNorm(), bNorm, 0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());
        bn2 = new MagneticFluxDensity(bn, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(be, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(bd, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bn, triad1.getValueX(), 0.0);
        assertEquals(be, triad1.getValueY(), 0.0);
        assertEquals(bd, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density measurements
        bn1 = new MagneticFluxDensity(bn, MagneticFluxDensityUnit.TESLA);
        be1 = new MagneticFluxDensity(be, MagneticFluxDensityUnit.TESLA);
        bd1 = new MagneticFluxDensity(bd, MagneticFluxDensityUnit.TESLA);
        b = new NEDMagneticFluxDensity(bn1, be1, bd1);

        // check default values
        assertEquals(bn, b.getBn(), 0.0);
        assertEquals(be, b.getBe(), 0.0);
        assertEquals(bd, b.getBd(), 0.0);
        assertEquals(bNorm, b.getNorm(), 0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());
        bn2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bn, triad1.getValueX(), 0.0);
        assertEquals(be, triad1.getValueY(), 0.0);
        assertEquals(bd, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test constructor with magnetic flux density triad
        final var triad = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, bn, be, bd);
        b = new NEDMagneticFluxDensity(triad);

        // check default values
        assertEquals(bn, b.getBn(), 0.0);
        assertEquals(be, b.getBe(), 0.0);
        assertEquals(bd, b.getBd(), 0.0);
        assertEquals(bNorm, b.getNorm(), 0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());
        bn2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bn, triad1.getValueX(), 0.0);
        assertEquals(be, triad1.getValueY(), 0.0);
        assertEquals(bd, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);

        // test copy constructor
        final var b2 = new NEDMagneticFluxDensity(b);

        assertEquals(bn, b2.getBn(), 0.0);
        assertEquals(be, b2.getBe(), 0.0);
        assertEquals(bd, b2.getBd(), 0.0);
        assertEquals(b2.getNorm(), b.getNorm(), 0.0);
        bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(bn, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());
        be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(be, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());
        bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(bd, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());
        bn2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn2);
        assertEquals(bn1, bn2);
        be2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be2);
        assertEquals(be1, be2);
        bd2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd2);
        assertEquals(bd1, bd2);
        triad1 = b.getCoordinatesAsTriad();
        assertEquals(bn, triad1.getValueX(), 0.0);
        assertEquals(be, triad1.getValueY(), 0.0);
        assertEquals(bd, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        triad2 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad2);
        assertEquals(triad1, triad2);
    }

    @Test
    void testGetSetBn() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBn(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBn(bn);

        // check
        assertEquals(bn, b.getBn(), 0.0);
    }

    @Test
    void testGetSetBnAsMagneticFluxDensity() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        final var bn1 = b.getBnAsMagneticFluxDensity();
        assertEquals(0.0, bn1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bn1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var bn2 = new MagneticFluxDensity(bn, MagneticFluxDensityUnit.TESLA);
        b.setBn(bn2);

        // check
        final var bn3 = b.getBnAsMagneticFluxDensity();
        final var bn4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBnAsMagneticFluxDensity(bn4);

        assertEquals(bn2, bn3);
        assertEquals(bn2, bn4);
    }

    @Test
    void testGetSetBe() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBe(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBe(be);

        // check
        assertEquals(be, b.getBe(), 0.0);
    }

    @Test
    void testGetSetBeAsMagneticFluxDensity() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        final var be1 = b.getBeAsMagneticFluxDensity();
        assertEquals(0.0, be1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, be1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var be2 = new MagneticFluxDensity(be, MagneticFluxDensityUnit.TESLA);
        b.setBe(be2);

        // check
        final var be3 = b.getBeAsMagneticFluxDensity();
        final var be4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBeAsMagneticFluxDensity(be4);

        assertEquals(be2, be3);
        assertEquals(be2, be4);
    }

    @Test
    void testGetSetBd() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getBd(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setBd(bd);

        // check
        assertEquals(bd, b.getBd(), 0.0);
    }

    @Test
    void testGetSetBdAsMagneticFluxDensity() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        final var bd1 = b.getBdAsMagneticFluxDensity();
        assertEquals(0.0, bd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bd1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var bd2 = new MagneticFluxDensity(bd, MagneticFluxDensityUnit.TESLA);
        b.setBd(bd2);

        // check
        final var bd3 = b.getBdAsMagneticFluxDensity();
        final var bd4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getBdAsMagneticFluxDensity(bd4);

        assertEquals(bd2, bd3);
        assertEquals(bd2, bd4);
    }

    @Test
    void testSetCoordinates1() {
        final var b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBn(), 0.0);
        assertEquals(0.0, b.getBe(), 0.0);
        assertEquals(0.0, b.getBd(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check default values
        assertEquals(bn, b.getBn(), 0.0);
        assertEquals(be, b.getBe(), 0.0);
        assertEquals(bd, b.getBd(), 0.0);
    }

    @Test
    void testSetCoordinates2() {
        final var b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBn(), 0.0);
        assertEquals(0.0, b.getBe(), 0.0);
        assertEquals(0.0, b.getBd(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(new MagneticFluxDensity(bn, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(be, MagneticFluxDensityUnit.TESLA),
                new MagneticFluxDensity(bd, MagneticFluxDensityUnit.TESLA));

        // check default values
        assertEquals(bn, b.getBn(), 0.0);
        assertEquals(be, b.getBe(), 0.0);
        assertEquals(bd, b.getBd(), 0.0);
    }

    @Test
    void testSetCoordinates3() {
        final var b = new NEDMagneticFluxDensity();

        // check default values
        assertEquals(0.0, b.getBn(), 0.0);
        assertEquals(0.0, b.getBe(), 0.0);
        assertEquals(0.0, b.getBd(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var triad1 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.TESLA, bn, be, bd);
        b.setCoordinates(triad1);

        // check default values
        assertEquals(bn, b.getBn(), 0.0);
        assertEquals(be, b.getBe(), 0.0);
        assertEquals(bd, b.getBd(), 0.0);
        final var triad2 = b.getCoordinatesAsTriad();
        final var triad3 = new MagneticFluxDensityTriad();
        b.getCoordinatesAsTriad(triad3);
        assertEquals(triad1, triad2);
        assertEquals(triad1, triad3);
    }

    @Test
    void testGetNorm() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        assertEquals(0.0, b.getNorm(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check
        assertEquals(Math.sqrt(bn * bn + be * be + bd * bd), b.getNorm(), 0.0);
    }

    @Test
    void testGetNormAsMagneticFluxDensity() {
        final var b = new NEDMagneticFluxDensity();

        // check default value
        final var norm1 = b.getNormAsMagneticFluxDensity();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        b.setCoordinates(bn, be, bd);

        // check
        final var norm = Math.sqrt(bn * bn + be * be + bd * bd);
        final var norm2 = b.getNormAsMagneticFluxDensity();
        assertEquals(norm2.getValue().doubleValue(), norm, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm2.getUnit());
        final var norm3 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        b.getNormAsMagneticFluxDensity(norm3);
        assertEquals(norm2, norm3);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b2 = new NEDMagneticFluxDensity();

        b1.copyTo(b2);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b2 = new NEDMagneticFluxDensity();

        b2.copyFrom(b1);

        // check
        assertEquals(b1.getBn(), b2.getBn(), 0.0);
        assertEquals(b1.getBe(), b2.getBe(), 0.0);
        assertEquals(b1.getBd(), b2.getBd(), 0.0);
    }

    @Test
    void testAsArray() {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b = new NEDMagneticFluxDensity(bn, be, bd);

        final var array1 = b.asArray();
        final var array2 = new double[NEDMagneticFluxDensity.COMPONENTS];
        b.asArray(array2);

        // check
        assertEquals(bn, array1[0], 0.0);
        assertEquals(be, array1[1], 0.0);
        assertEquals(bd, array1[2], 0.0);
        assertArrayEquals(array1, array2, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> b.asArray(new double[1]));
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b = new NEDMagneticFluxDensity(bn, be, bd);

        final var m1 = b.asMatrix();
        final var m2 = new Matrix(1, 1);
        b.asMatrix(m2);
        final var m3 = new Matrix(NEDMagneticFluxDensity.COMPONENTS, 1);
        b.asMatrix(m3);

        // check
        assertEquals(bn, m1.getElementAtIndex(0), 0.0);
        assertEquals(be, m1.getElementAtIndex(1), 0.0);
        assertEquals(bd, m1.getElementAtIndex(2), 0.0);
        assertEquals(m1, m2);
        assertEquals(m1, m3);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b2 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b3 = new NEDMagneticFluxDensity();

        assertEquals(b1.hashCode(), b2.hashCode());
        assertNotEquals(b1.hashCode(), b3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b2 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b3 = new NEDMagneticFluxDensity();

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
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b2 = new NEDMagneticFluxDensity(bn, be, bd);
        final var b3 = new NEDMagneticFluxDensity();

        assertTrue(b1.equals(b1, THRESHOLD));
        assertTrue(b1.equals(b2, THRESHOLD));
        assertFalse(b1.equals(b3, THRESHOLD));
        assertFalse(b1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);

        final var b2 = b1.clone();

        // check
        assertEquals(b1, b2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var bn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var be = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var bd = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var b1 = new NEDMagneticFluxDensity(bn, be, bd);

        final var bytes = SerializationHelper.serialize(b1);
        final var b2 = SerializationHelper.deserialize(bytes);

        assertEquals(b1, b2);
        assertNotSame(b1, b2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = NEDMagneticFluxDensity.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
