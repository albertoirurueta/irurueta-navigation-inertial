/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;

import static org.junit.jupiter.api.Assertions.*;

class MagneticFluxDensityFixerTest {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    void testConstructor() throws WrongSizeException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(new Matrix(3, 1), fixer.getBias());
        final var b = new Matrix(3, 1);
        fixer.getBias(b);
        assertEquals(new Matrix(3, 1), b);

        assertArrayEquals(new double[3], fixer.getBiasArray(), 0.0);
        final var b2 = new double[3];
        fixer.getBiasArray(b2);
        assertArrayEquals(new double[3], b2, 0.0);
        assertEquals(0.0, fixer.getBiasX(), 0.0);
        assertEquals(0.0, fixer.getBiasY(), 0.0);
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        final var bb1 = fixer.getBiasAsBodyMagneticFluxDensity();
        assertEquals(0.0, bb1.getBx(), 0.0);
        assertEquals(0.0, bb1.getBy(), 0.0);
        assertEquals(0.0, bb1.getBz(), 0.0);
        final var bb2 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(bb2);
        assertEquals(bb1, bb2);

        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final var triad2 = new MagneticFluxDensityTriad();
        fixer.getBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(new Matrix(3, 3), fixer.getCrossCouplingErrors());
        final var m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(new Matrix(3, 3), m);

        final var bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        final var bx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasXAsMagneticFluxDensity(bx2);
        assertEquals(bx1, bx2);
        final var by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        final var by2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasYAsMagneticFluxDensity(by2);
        assertEquals(by1, by2);
        final var bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());
        final var bz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasZAsMagneticFluxDensity(bz2);
        assertEquals(bz1, bz2);

        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);
    }

    @Test
    void testGetSetBias() throws WrongSizeException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var b1 = fixer.getBias();
        final var b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(new Matrix(3, 1), b1);
        assertEquals(b1, b2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b3 = Matrix.newFromArray(generateHardIron(randomizer));
        fixer.setBias(b3);

        // check
        final var b4 = fixer.getBias();
        final var b5 = new Matrix(3, 1);
        fixer.getBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setBias(m2));
    }

    @Test
    void testGetSetBiasArray() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var b1 = fixer.getBiasArray();
        final var b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(new double[3], b1, 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b3 = generateHardIron(randomizer);
        fixer.setBias(b3);

        // check
        final var b4 = fixer.getBiasArray();
        final var b5 = new double[3];
        fixer.getBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.getBiasArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> fixer.setBias(new double[1]));
    }

    @Test
    void testGetSetBiasAsBodyMagneticFluxDensity() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var b1 = fixer.getBiasAsBodyMagneticFluxDensity();
        final var b2 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(b2);

        assertEquals(0.0, b1.getBx(), 0.0);
        assertEquals(0.0, b1.getBy(), 0.0);
        assertEquals(0.0, b1.getBz(), 0.0);
        assertEquals(b1, b2);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var b3 = new BodyMagneticFluxDensity(b[0], b[1], b[2]);
        fixer.setBias(b3);

        // check
        final var b4 = fixer.getBiasAsBodyMagneticFluxDensity();
        final var b5 = new BodyMagneticFluxDensity();
        fixer.getBiasAsBodyMagneticFluxDensity(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);
    }

    @Test
    void testGetSetBiasTriad() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());

        // set new value
        final var triad2 = new MagneticFluxDensityTriad();
        final var randomizer = new UniformRandomizer();
        triad2.setValueCoordinates(generateHardIron(randomizer));
        fixer.setBias(triad2);

        // check
        final var triad3 = fixer.getBiasAsTriad();
        final var triad4 = new MagneticFluxDensityTriad();
        fixer.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetBiasX() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bx = b[0];
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    void testGetSetBiasY() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var by = b[1];
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    void testGetSetBiasZ() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bz = b[2];
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    void testSetBias1() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasX(), 0.0);
        assertEquals(0.0, fixer.getBiasY(), 0.0);
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bx = b[0];
        final var by = b[1];
        final var bz = b[2];
        fixer.setBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
        assertEquals(by, fixer.getBiasY(), 0.0);
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    void testGetSetBiasXAsMagneticFluxDensity() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bx = b[0];
        final var bx2 = new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasX(bx2);

        // check
        final var bx3 = fixer.getBiasXAsMagneticFluxDensity();
        final var bx4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasXAsMagneticFluxDensity(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetBiasYAsMagneticFluxDensity() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var by = b[1];
        final var by2 = new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasY(by2);

        // check
        final var by3 = fixer.getBiasYAsMagneticFluxDensity();
        final var by4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasYAsMagneticFluxDensity(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetBiasZAsMagneticFluxDensity() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        final var bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bz = b[2];
        final var bz2 = new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
        fixer.setBiasZ(bz2);

        // check
        final var bz3 = fixer.getBiasZAsMagneticFluxDensity();
        final var bz4 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        fixer.getBiasZAsMagneticFluxDensity(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testSetBias2() {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        final var bx1 = fixer.getBiasXAsMagneticFluxDensity();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        final var by1 = fixer.getBiasYAsMagneticFluxDensity();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        final var bz1 = fixer.getBiasZAsMagneticFluxDensity();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var bx = b[0];
        final var by = b[1];
        final var bz = b[2];
        final var bx2 = new MagneticFluxDensity(bx, MagneticFluxDensityUnit.TESLA);
        final var by2 = new MagneticFluxDensity(by, MagneticFluxDensityUnit.TESLA);
        final var bz2 = new MagneticFluxDensity(bz, MagneticFluxDensityUnit.TESLA);
        fixer.setBias(bx2, by2, bz2);

        // check
        final var bx3 = fixer.getBiasXAsMagneticFluxDensity();
        final var by3 = fixer.getBiasYAsMagneticFluxDensity();
        final var bz3 = fixer.getBiasZAsMagneticFluxDensity();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    void testGetSetCrossCouplingErrors() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        final var m1 = fixer.getCrossCouplingErrors();
        final var m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(new Matrix(3, 3), m1);
        assertEquals(m1, m2);

        // set new values
        final var m3 = generateSoftIronGeneral();
        assertNotNull(m3);
        final var sx = m3.getElementAt(0, 0);
        final var sy = m3.getElementAt(1, 1);
        final var sz = m3.getElementAt(2, 2);
        final var mxy = m3.getElementAt(0, 1);
        final var mxz = m3.getElementAt(0, 2);
        final var myx = m3.getElementAt(1, 0);
        final var myz = m3.getElementAt(1, 2);
        final var mzx = m3.getElementAt(2, 0);
        final var mzy = m3.getElementAt(2, 1);

        fixer.setCrossCouplingErrors(m3);

        // check
        final var m4 = fixer.getCrossCouplingErrors();
        final var m5 = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getSx(), 0.0);
        assertEquals(sy, fixer.getSy(), 0.0);
        assertEquals(sz, fixer.getSz(), 0.0);
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);

        // Force IllegalArgumentException
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setCrossCouplingErrors(m6));
        final var m7 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setCrossCouplingErrors(m7));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> fixer.setCrossCouplingErrors(wrong));
    }

    @Test
    void testGetSetSx() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getSx(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    void testGetSetSy() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getSy(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    void testGetSetSz() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    void testGetSetMxy() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMxy(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    void testGetSetMxz() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMxz(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    void testGetSetMyx() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMyx(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    void testGetSetMyz() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMyz(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    void testGetSetMzx() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMzx(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    void testGetSetMzy() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default value
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new value
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    void testSetScalingFactors() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set new values
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);

        fixer.setScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
        assertEquals(sy, fixer.getSy(), 0.0);
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    void testSetCrossCouplingErrors() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new values
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    void testSetScalingFactorsAndCrossCouplingErrors() throws AlgebraException {
        final var fixer = new MagneticFluxDensityFixer();

        // check default values
        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new values
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
        assertEquals(sy, fixer.getSy(), 0.0);
        assertEquals(sz, fixer.getSz(), 0.0);
        assertEquals(mxy, fixer.getMxy(), 0.0);
        assertEquals(mxz, fixer.getMxz(), 0.0);
        assertEquals(myx, fixer.getMyx(), 0.0);
        assertEquals(myz, fixer.getMyz(), 0.0);
        assertEquals(mzx, fixer.getMzx(), 0.0);
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    void testFix1() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, new double[1]));
    }

    @Test
    void testFix2() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m).getCoordinatesAsTriad();

        final var result = new double[MagneticFluxDensityTriad.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, new double[1]));
    }

    @Test
    void testFix3() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m2));
    }

    @Test
    void testFix4() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m).getCoordinatesAsTriad();

        final var result = new Matrix(MagneticFluxDensityTriad.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m2));
    }

    @Test
    void testFix5() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);

        final var result = new BodyMagneticFluxDensity();
        fixer.fix(measuredB, result);

        // check
        assertTrue(result.equals(trueB, ABSOLUTE_ERROR));
    }

    @Test
    void testFix6() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m).getCoordinatesAsTriad();

        final var result = new MagneticFluxDensityTriad();
        fixer.fix(measuredB, result);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    void testFix7() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measuredB.getBxAsMagneticFluxDensity();
        final var measuredBy = measuredB.getByAsMagneticFluxDensity();
        final var measuredBz = measuredB.getBzAsMagneticFluxDensity();

        final var result = new BodyMagneticFluxDensity();
        fixer.fix(measuredBx, measuredBy, measuredBz, result);

        // check
        assertTrue(result.equals(trueB, ABSOLUTE_ERROR));
    }

    @Test
    void testFix8() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measuredB.getBxAsMagneticFluxDensity();
        final var measuredBy = measuredB.getByAsMagneticFluxDensity();
        final var measuredBz = measuredB.getBzAsMagneticFluxDensity();

        final var result = new MagneticFluxDensityTriad();
        fixer.fix(measuredBx, measuredBy, measuredBz, result);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    void testFix9() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asArray();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, new double[1]));
    }

    @Test
    void testFix10() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asMatrix();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, new double[1]));
    }

    @Test
    void testFix11() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asMatrix();

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m4));
    }

    @Test
    void testFix12() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measuredB.getBx();
        final var measBy = measuredB.getBy();
        final var measBz = measuredB.getBz();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, new double[1]));
    }

    @Test
    void testFix13() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measuredB.getBx();
        final var measBy = measuredB.getBy();
        final var measBz = measuredB.getBz();

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, m2));
    }

    @Test
    void testFix14() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asArray();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], b, m, result));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m1, m, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m2, m, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m3, result));
        final var m4 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m4, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m, new double[1]));
    }

    @Test
    void testFix15() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asMatrix();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, b, m, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, b, m, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m3, m, result));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m4, m, result));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m5, result));
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m6, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m, new double[1]));
    }

    @Test
    void testFix16() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asMatrix();

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measuredB, b, m, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, b, m, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, b, m, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m3, m, result));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, m4, m, result));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m5, result));
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m6, result));
        final var m7 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m, m7));
        final var m8 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredB, b, m, m8));
    }

    @Test
    void testFix17() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measB.getBx();
        final var measBy = measB.getBy();
        final var measBz = measB.getBz();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m1,
                result));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m2,
                result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                new double[1]));
    }

    @Test
    void testFix18() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measB.getBx();
        final var measBy = measB.getBy();
        final var measBz = measB.getBz();

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m1,
                result));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m2,
                result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                m3));
        final var m4 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, m,
                m4));
    }

    @Test
    void testFix19() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measB.getBx();
        final var measBy = measB.getBy();
        final var measBz = measB.getBz();

        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);

        // check
        assertEquals(trueBx, result[0], ABSOLUTE_ERROR);
        assertEquals(trueBy, result[1], ABSOLUTE_ERROR);
        assertEquals(trueBz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, new double[1]));
    }

    @Test
    void testFix20() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var trueBx = trueB.getBx();
        final var trueBy = trueB.getBy();
        final var trueBz = trueB.getBz();

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measBx = measB.getBx();
        final var measBy = measB.getBy();
        final var measBz = measB.getBz();

        final var result = new Matrix(BodyMagneticFluxDensity.COMPONENTS, 1);
        fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, result);

        // check
        assertEquals(trueBx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(trueBy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(trueBz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measBx, measBy, measBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, m2));
    }

    @Test
    void testFixAndReturnNew1() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m).getCoordinatesAsTriad();

        final var result = fixer.fixAndReturnNew(measuredB);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew2() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measuredB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measuredB.getBxAsMagneticFluxDensity();
        final var measuredBy = measuredB.getByAsMagneticFluxDensity();
        final var measuredBz = measuredB.getBzAsMagneticFluxDensity();

        final var result = fixer.fixAndReturnNew(measuredBx, measuredBy, measuredBz);

        // check
        assertTrue(result.equals(trueB.getCoordinatesAsTriad(), ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew3() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asArray();

        final var result = fixer.fixAndReturnNew(measuredB);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(new double[1]));
    }

    @Test
    void testFixAndReturnNew4() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asMatrix();

        final var result = fixer.fixAndReturnNew(measuredB);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m2));
    }

    @Test
    void testFixAndReturnNewMatrix1() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredB = measB.asMatrix();

        final var result = fixer.fixAndReturnNewMatrix(measuredB);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m2));
    }

    @Test
    void testFixAndReturnNew5() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNew(measuredBx, measuredBy, measuredBz);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix2() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNewMatrix(measuredBx, measuredBy, measuredBz);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew6() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asArray();

        final var result = fixer.fixAndReturnNew(measuredB, b, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew7() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asMatrix();

        final var result = fixer.fixAndReturnNew(measuredB, b, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix3() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = Matrix.newFromArray(generateHardIron(randomizer));
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b.getBuffer(), m);
        final var measuredB = measB.asMatrix();

        final var result = fixer.fixAndReturnNewMatrix(measuredB, b, m);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew8() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNew(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ, m);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix4() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNewMatrix(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ, m);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew9() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNew(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertArrayEquals(result, trueB.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix5() throws AlgebraException, IOException {
        final var fixer = new MagneticFluxDensityFixer();

        final var randomizer = new UniformRandomizer();
        final var b = generateHardIron(randomizer);
        final var biasX = b[0];
        final var biasY = b[1];
        final var biasZ = b[2];
        final var m = generateSoftIronGeneral();
        assertNotNull(m);
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setBias(b);
        fixer.setCrossCouplingErrors(m);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));
        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var trueB = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);

        final var measB = BodyMagneticFluxDensityGenerator.generate(trueB, b, m);
        final var measuredBx = measB.getBx();
        final var measuredBy = measB.getBy();
        final var measuredBz = measB.getBz();

        final var result = fixer.fixAndReturnNewMatrix(measuredBx, measuredBy, measuredBz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertTrue(result.equals(trueB.asMatrix(), ABSOLUTE_ERROR));
    }


    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
