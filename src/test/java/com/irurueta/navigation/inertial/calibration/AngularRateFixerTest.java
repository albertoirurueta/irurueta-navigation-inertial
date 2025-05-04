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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AngularRateFixerTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testConstructor() throws WrongSizeException {
        final var fixer = new AngularRateFixer();

        // check default values
        assertEquals(new Matrix(3, 1), fixer.getBias());
        final var b = new Matrix(3, 1);
        fixer.getBias(b);
        assertEquals(new Matrix(3, 1), b);

        assertArrayEquals(new double[3], fixer.getBiasArray(), 0.0);
        final var b2 = new double[3];
        fixer.getBiasArray(b2);
        assertArrayEquals(new double[3], b2, 0.0);

        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final var triad2 = new AngularSpeedTriad();
        fixer.getBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, fixer.getBiasX(), 0.0);
        assertEquals(0.0, fixer.getBiasY(), 0.0);
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        final var bx1 = fixer.getBiasXAsAngularSpeed();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());
        final var bx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasXAsAngularSpeed(bx2);
        assertEquals(bx1, bx2);
        final var by1 = fixer.getBiasYAsAngularSpeed();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());
        final var by2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasYAsAngularSpeed(by2);
        assertEquals(by1, by2);
        final var bz1 = fixer.getBiasZAsAngularSpeed();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());
        final var bz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasZAsAngularSpeed(bz2);
        assertEquals(bz1, bz2);

        assertEquals(new Matrix(3, 3), fixer.getCrossCouplingErrors());
        final var m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(new Matrix(3, 3), m);

        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);

        assertEquals(new Matrix(3, 3), fixer.getGDependantCrossBias());
        final var g = new Matrix(3, 3);
        fixer.getGDependantCrossBias(g);
        assertEquals(new Matrix(3, 3), g);
    }

    @Test
    public void testGetSetBias() throws WrongSizeException {
        final var fixer = new AngularRateFixer();

        // check default value
        final var b1 = fixer.getBias();
        final var b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(new Matrix(3, 1), b1);
        assertEquals(b1, b2);

        // set new value
        final var b3 = generateBg();
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
    public void testGetSetBiasArray() {
        final var fixer = new AngularRateFixer();

        // check default value
        final var b1 = fixer.getBiasArray();
        final var b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(new double[3], b1, 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final var b3 = generateBg().getBuffer();
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
    public void testGetSetBiasTriad() {
        final var fixer = new AngularRateFixer();

        // check default value
        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final var triad2 = new AngularSpeedTriad();
        triad2.setValueCoordinates(generateBg());
        fixer.setBias(triad2);

        // check
        final var triad3 = fixer.getBiasAsTriad();
        final var triad4 = new AngularSpeedTriad();
        fixer.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetBiasX() {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasX(), 0.0);

        // set new value
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    public void testGetSetBiasY() {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasY(), 0.0);

        // set new value
        final var b = generateBg();
        final var by = b.getElementAtIndex(1);
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    public void testGetSetBiasZ() {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set new value
        final var b = generateBg();
        final var bz = b.getElementAtIndex(2);
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBias1() {
        final var fixer = new AngularRateFixer();

        // check default values
        assertEquals(0.0, fixer.getBiasX(), 0.0);
        assertEquals(0.0, fixer.getBiasY(), 0.0);
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set new values
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        final var by = b.getElementAtIndex(1);
        final var bz = b.getElementAtIndex(2);
        fixer.setBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
        assertEquals(by, fixer.getBiasY(), 0.0);
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetBiasXAsAngularSpeed() {
        final var fixer = new AngularRateFixer();

        // check default value
        final var bx1 = fixer.getBiasXAsAngularSpeed();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        // set new value
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        final var bx2 = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setBiasX(bx2);

        // check
        final var bx3 = fixer.getBiasXAsAngularSpeed();
        final var bx4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasXAsAngularSpeed(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetBiasYAsAngularSpeed() {
        final var fixer = new AngularRateFixer();

        // check default value
        final var by1 = fixer.getBiasYAsAngularSpeed();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        // set new value
        final var b = generateBg();
        final var by = b.getElementAtIndex(1);
        final var by2 = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setBiasY(by2);

        // check
        final var by3 = fixer.getBiasYAsAngularSpeed();
        final var by4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasYAsAngularSpeed(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetBiasZAsAngularSpeed() {
        final var fixer = new AngularRateFixer();

        // check default value
        final var bz1 = fixer.getBiasZAsAngularSpeed();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new value
        final var b = generateBg();
        final var bz = b.getElementAtIndex(2);
        final var bz2 = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setBiasZ(bz2);

        // check
        final var bz3 = fixer.getBiasZAsAngularSpeed();
        final var bz4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getBiasZAsAngularSpeed(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testGetSetBias2() {
        final var fixer = new AngularRateFixer();

        // check default values
        final var bx1 = fixer.getBiasXAsAngularSpeed();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        final var by1 = fixer.getBiasYAsAngularSpeed();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        final var bz1 = fixer.getBiasZAsAngularSpeed();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new values
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        final var by = b.getElementAtIndex(1);
        final var bz = b.getElementAtIndex(2);
        final var bx2 = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var by2 = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bz2 = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setBias(bx2, by2, bz2);

        // check
        final var bx3 = fixer.getBiasXAsAngularSpeed();
        final var by3 = fixer.getBiasYAsAngularSpeed();
        final var bz3 = fixer.getBiasZAsAngularSpeed();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    public void testGetSetCrossCouplingErrors() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default values
        final var m1 = fixer.getCrossCouplingErrors();
        final var m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(new Matrix(3, 3), m1);
        assertEquals(m1, m2);

        // set new values
        final var m3 = generateMg();
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
    public void testGetSetSx() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getSx(), 0.0);

        // set new value
        final var m = generateMg();
        final var sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    public void testGetSetSy() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getSy(), 0.0);

        // set new value
        final var m = generateMg();
        final var sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    public void testGetSetSz() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set new value
        final var m = generateMg();
        final var sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    public void testGetSetMxy() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMxy(), 0.0);

        // set new value
        final var m = generateMg();
        final var mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    public void testGetSetMxz() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMxz(), 0.0);

        // set new value
        final var m = generateMg();
        final var mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    public void testGetSetMyx() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMyx(), 0.0);

        // set new value
        final var m = generateMg();
        final var myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    public void testGetSetMyz() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMyz(), 0.0);

        // set new value
        final var m = generateMg();
        final var myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    public void testGetSetMzx() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMzx(), 0.0);

        // set new value
        final var m = generateMg();
        final var mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    public void testGetSetMzy() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default value
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new value
        final var m = generateMg();
        final var mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    public void testSetScalingFactors() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default values
        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set new values
        final var m = generateMg();
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
    public void testSetCrossCouplingErrors() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default values
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new values
        final var m = generateMg();
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
    public void testSetScalingFactorsAndCrossCouplingErrors() throws AlgebraException {
        final var fixer = new AngularRateFixer();

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
        final var m = generateMg();
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
    public void testGetSetGDependantCrossBias() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        // check default values
        final var g1 = fixer.getGDependantCrossBias();
        final var g2 = new Matrix(1, 1);
        fixer.getGDependantCrossBias(g2);

        assertEquals(new Matrix(3, 3), g1);
        assertEquals(g1, g2);

        // set new values
        final var g3 = generateGg();

        fixer.setGDependantCrossBias(g3);

        // check
        final var g4 = fixer.getGDependantCrossBias();
        final var g5 = new Matrix(3, 3);
        fixer.getGDependantCrossBias(g5);

        assertEquals(g3, g4);
        assertEquals(g3, g5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setGDependantCrossBias(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setGDependantCrossBias(m2));
    }

    @Test
    public void testFix1() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.getAngularRateTriad();
        final var trueF = trueKinematics.getSpecificForceTriad();

        final var result = new double[AngularSpeedTriad.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, result);

        // check
        assertEquals(result[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, new double[1]));
    }

    @Test
    public void testFix2() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.getAngularRateTriad();
        final var trueF = trueKinematics.getSpecificForceTriad();

        final var result = new Matrix(AngularSpeedTriad.COMPONENTS, 1);
        fixer.fix(measuredAngularRate, trueF, result);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m1));
        final var m2 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m2));
    }

    @Test
    public void testFix3() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.getAngularRateTriad();
        final var trueF = trueKinematics.getSpecificForceTriad();

        final var result = new AngularSpeedTriad();
        fixer.fix(measuredAngularRate, trueF, result);

        // check
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.getUnit());
        assertEquals(omegaX, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFix4() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularSpeedX();
        final var measuredAngularSpeedY = measuredKinematics.getAngularSpeedY();
        final var measuredAngularSpeedZ = measuredKinematics.getAngularSpeedZ();
        final var trueFx = trueKinematics.getSpecificForceX();
        final var trueFy = trueKinematics.getSpecificForceY();
        final var trueFz = trueKinematics.getSpecificForceZ();

        final var result = new AngularSpeedTriad();
        fixer.fix(measuredAngularRateX, measuredAngularSpeedY, measuredAngularSpeedZ, trueFx, trueFy, trueFz, result);

        // check
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.getUnit());
        assertEquals(omegaX, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFix5() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateArray();
        final var trueF = trueKinematics.asSpecificForceArray();

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(result1[0], omegaX, ABSOLUTE_ERROR);
        assertEquals(result1[1], omegaY, ABSOLUTE_ERROR);
        assertEquals(result1[2], omegaZ, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], trueF, result1));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, new double[1], result1));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, new double[1]));
    }

    @Test
    public void testFix6() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(omegaX, result1[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result1[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, trueF, result1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, trueF, result1));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m3, result1));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m4, result1));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, new double[1]));
    }

    @Test
    public void testFix7() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRate, trueF, result1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(omegaX, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, trueF, result1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, trueF, result1));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m3, result1));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m4, result1));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m5));
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m6));
    }

    @Test
    public void testFix8() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, mg, gg, result1);
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(omegaX, result1[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result1[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRateX, measuredAngularRateY,
                measuredAngularRateZ, trueFx, trueFy, trueFz, new double[1]));
    }

    @Test
    public void testFix9() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, result1);
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, mg, gg, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(omegaX, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRateX, measuredAngularRateY,
                measuredAngularRateZ, trueFx, trueFy, trueFz, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRateX, measuredAngularRateY,
                measuredAngularRateZ, trueFx, trueFy, trueFz, m2));
    }

    @Test
    public void testFix10() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateArray();
        final var trueF = trueKinematics.asSpecificForceArray();

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], trueF, bg, mg, gg, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, new double[1], bg, mg, gg,
                result));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m1, mg, gg, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m2, mg, gg, result));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m3, gg, result));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m4, gg, result));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m5, result));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m6, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, gg,
                new double[1]));
    }

    @Test
    public void testFix11() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, trueF, bg, mg, gg, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, trueF, bg, mg, gg, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m3, bg, mg, gg, result));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m4, bg, mg, gg, result));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m5, mg, gg, result));
        final var m6 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m6, mg, gg, result));
        final var m7 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m7, gg, result));
        final var m8 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m8, gg, result));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m9, result));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m10, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, gg,
                new double[1]));
    }

    @Test
    public void testFix12() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRate, trueF, bg, mg, gg, result);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, trueF, bg, mg, gg, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, trueF, bg, mg, gg, result));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m3, bg, mg, gg, result));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, m4, bg, mg, gg, result));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m5, mg, gg, result));
        final var m6 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, m6, mg, gg, result));
        final var m7 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m7, gg, result));
        final var m8 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, m8, gg, result));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m9, result));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, m10, result));
        final var m11 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, gg, m11));
        final var m12 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredAngularRate, trueF, bg, mg, gg, m12));
    }

    @Test
    public void testFix13() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, mg, gg, result);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m1, gg, result));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m2, gg, result));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m3, result));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m4, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, new double[1]));
    }

    @Test
    public void testFix14() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, mg, gg, result);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m1, gg, result));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m2, gg, result));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m3, result));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m4, result));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, m5));
        final var m6 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, gg, m6));
    }

    @Test
    public void testFix15() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var myx = mg.getElementAt(1, 0);
        final var mzx = mg.getElementAt(2, 0);
        final var mxy = mg.getElementAt(0, 1);
        final var sy = mg.getElementAt(1, 1);
        final var mzy = mg.getElementAt(2, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myz = mg.getElementAt(1, 2);
        final var sz = mg.getElementAt(2, 2);

        final var g11 = gg.getElementAt(0, 0);
        final var g21 = gg.getElementAt(1, 0);
        final var g31 = gg.getElementAt(2, 0);
        final var g12 = gg.getElementAt(0, 1);
        final var g22 = gg.getElementAt(1, 1);
        final var g32 = gg.getElementAt(2, 1);
        final var g13 = gg.getElementAt(0, 2);
        final var g23 = gg.getElementAt(1, 2);
        final var g33 = gg.getElementAt(2, 2);

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31, g12, g22, g32, g13, g23, g33,
                result);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31, g12, g22, g32, g13, g23, g33,
                new double[1]));
    }

    @Test
    public void testFix16() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var myx = mg.getElementAt(1, 0);
        final var mzx = mg.getElementAt(2, 0);
        final var mxy = mg.getElementAt(0, 1);
        final var sy = mg.getElementAt(1, 1);
        final var mzy = mg.getElementAt(2, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myz = mg.getElementAt(1, 2);
        final var sz = mg.getElementAt(2, 2);

        final var g11 = gg.getElementAt(0, 0);
        final var g21 = gg.getElementAt(1, 0);
        final var g31 = gg.getElementAt(2, 0);
        final var g12 = gg.getElementAt(0, 1);
        final var g22 = gg.getElementAt(1, 1);
        final var g32 = gg.getElementAt(2, 1);
        final var g13 = gg.getElementAt(0, 2);
        final var g23 = gg.getElementAt(1, 2);
        final var g33 = gg.getElementAt(2, 2);

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz,
                bgx, bgy, bgz, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31, g12, g22, g32, g13, g23, g33,
                result);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31, g12, g22, g32, g13, g23, g33, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31, g12, g22, g32, g13, g23, g33, m2));
    }

    @Test
    public void testFixAndReturnNew1() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.getAngularRateTriad();
        final var trueF = trueKinematics.getSpecificForceTriad();

        final var result = fixer.fixAndReturnNew(measuredAngularRate, trueF);

        // check
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.getUnit());
        assertEquals(omegaX, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew2() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularSpeedX();
        final var measuredAngularSpeedY = measuredKinematics.getAngularSpeedY();
        final var measuredAngularSpeedZ = measuredKinematics.getAngularSpeedZ();
        final var trueFx = trueKinematics.getSpecificForceX();
        final var trueFy = trueKinematics.getSpecificForceY();
        final var trueFz = trueKinematics.getSpecificForceZ();

        final var result = fixer.fixAndReturnNew(measuredAngularRateX, measuredAngularSpeedY, measuredAngularSpeedZ,
                trueFx, trueFy, trueFz);

        // check
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, result.getUnit());
        assertEquals(omegaX, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew3() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateArray();
        final var trueF = trueKinematics.asSpecificForceArray();

        final var result1 = fixer.fixAndReturnNew(measuredAngularRate, trueF);
        final var result2 = fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(omegaX, result1[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result1[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(new double[1], trueF));
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, new double[1]));
    }

    @Test
    public void testFixAndReturnNew4() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result1 = fixer.fixAndReturnNew(measuredAngularRate, trueF);
        final var result2 = fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(omegaX, result1[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result1[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m1, trueF));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m2, trueF));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, m4));
    }

    @Test
    public void testFixAndReturnNewMatrix1() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result1 = fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF);
        final var result2 = fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(result1, result2);
        assertEquals(omegaX, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m1, trueF));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m2, trueF));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, m4));
    }

    @Test
    public void testFixAndReturnNew5() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result1 = fixer.fixAndReturnNew(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz);
        final var result2 = fixer.fixAndReturnNew(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz, mg, gg);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(omegaX, result1[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result1[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1[2], ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix2() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(bg);
        fixer.setCrossCouplingErrors(mg);
        fixer.setGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result1 = fixer.fixAndReturnNewMatrix(measuredAngularRateX, measuredAngularRateY,
                measuredAngularRateZ, trueFx, trueFy, trueFz);
        final var result2 = fixer.fixAndReturnNewMatrix(measuredAngularRateX, measuredAngularRateY,
                measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz, mg, gg);

        // check
        assertEquals(result1, result2);
        assertEquals(omegaX, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result1.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNew6() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateArray();
        final var trueF = trueKinematics.asSpecificForceArray();

        final var result = fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(new double[1], trueF, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, new double[1],
                bg, mg, gg));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, m1, mg,
                gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, m2, mg,
                gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, m3,
                gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, m4,
                gg));
        final var m5 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg,
                m5));
        final var m6 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg,
                m6));
    }

    @Test
    public void testFixAndReturnNew7() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result = fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m1, trueF, bg, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m2, trueF, bg, mg, gg));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, m3, bg, mg, gg));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, m4, bg, mg, gg));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, m5, mg,
                gg));
        final var m6 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, m6, mg,
                gg));
        final var m7 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, m7,
                gg));
        final var m8 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, m8,
                gg));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg,
                m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredAngularRate, trueF, bg, mg,
                m10));
    }

    @Test
    public void testFixAndReturnNewMatrix3() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRate = measuredKinematics.asAngularRateMatrix();
        final var trueF = trueKinematics.asSpecificForceMatrix();

        final var result = fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg, mg, gg);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m1, trueF, bg, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(m2, trueF, bg, mg, gg));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, m3, bg, mg,
                gg));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, m4, bg, mg,
                gg));
        final var m5 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, m5,
                mg, gg));
        final var m6 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, m6,
                mg, gg));
        final var m7 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg,
                m7, gg));
        final var m8 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg,
                m8, gg));
        final var m9 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg,
                mg, m9));
        final var m10 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(measuredAngularRate, trueF, bg,
                mg, m10));
    }

    @Test
    public void testFixAndReturnNew8() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result = fixer.fixAndReturnNew(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz, mg, gg);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m4));
    }

    @Test
    public void testFixAndReturnNewMatrix4() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var result = fixer.fixAndReturnNewMatrix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz, mg, gg);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m1, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                m2, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNewMatrix(
                measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ, trueFx, trueFy, trueFz, bgx, bgy, bgz,
                mg, m4));
    }

    @Test
    public void testFixAndReturnNew9() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var myx = mg.getElementAt(1, 0);
        final var mzx = mg.getElementAt(2, 0);
        final var mxy = mg.getElementAt(0, 1);
        final var sy = mg.getElementAt(1, 1);
        final var mzy = mg.getElementAt(2, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myz = mg.getElementAt(1, 2);
        final var sz = mg.getElementAt(2, 2);

        final var g11 = gg.getElementAt(0, 0);
        final var g21 = gg.getElementAt(1, 0);
        final var g31 = gg.getElementAt(2, 0);
        final var g12 = gg.getElementAt(0, 1);
        final var g22 = gg.getElementAt(1, 1);
        final var g32 = gg.getElementAt(2, 1);
        final var g13 = gg.getElementAt(0, 2);
        final var g23 = gg.getElementAt(1, 2);
        final var g33 = gg.getElementAt(2, 2);

        final var result = fixer.fixAndReturnNew(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31,
                g12, g22, g32, g13, g23, g33);

        // check
        assertEquals(omegaX, result[0], ABSOLUTE_ERROR);
        assertEquals(omegaY, result[1], ABSOLUTE_ERROR);
        assertEquals(omegaZ, result[2], ABSOLUTE_ERROR);
    }

    @Test
    public void testFixAndReturnNewMatrix5() throws AlgebraException {
        final var fixer = new AngularRateFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer(new Random());
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredAngularRateX = measuredKinematics.getAngularRateX();
        final var measuredAngularRateY = measuredKinematics.getAngularRateY();
        final var measuredAngularRateZ = measuredKinematics.getAngularRateZ();
        final var trueFx = trueKinematics.getFx();
        final var trueFy = trueKinematics.getFy();
        final var trueFz = trueKinematics.getFz();

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var sx = mg.getElementAt(0, 0);
        final var myx = mg.getElementAt(1, 0);
        final var mzx = mg.getElementAt(2, 0);
        final var mxy = mg.getElementAt(0, 1);
        final var sy = mg.getElementAt(1, 1);
        final var mzy = mg.getElementAt(2, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myz = mg.getElementAt(1, 2);
        final var sz = mg.getElementAt(2, 2);

        final var g11 = gg.getElementAt(0, 0);
        final var g21 = gg.getElementAt(1, 0);
        final var g31 = gg.getElementAt(2, 0);
        final var g12 = gg.getElementAt(0, 1);
        final var g22 = gg.getElementAt(1, 1);
        final var g32 = gg.getElementAt(2, 1);
        final var g13 = gg.getElementAt(0, 2);
        final var g23 = gg.getElementAt(1, 2);
        final var g33 = gg.getElementAt(2, 2);

        final var result = fixer.fixAndReturnNewMatrix(measuredAngularRateX, measuredAngularRateY, measuredAngularRateZ,
                trueFx, trueFy, trueFz, bgx, bgy, bgz, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, g11, g21, g31,
                g12, g22, g32, g13, g23, g33);

        // check
        assertEquals(omegaX, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(omegaY, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(omegaZ, result.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMa() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }
}
