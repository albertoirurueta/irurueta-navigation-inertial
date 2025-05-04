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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AccelerationFixerTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    void testConstructor() throws WrongSizeException {
        final var fixer = new AccelerationFixer();

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

        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        fixer.getBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(new Matrix(3, 3), fixer.getCrossCouplingErrors());
        final var m = new Matrix(3, 3);
        fixer.getCrossCouplingErrors(m);
        assertEquals(new Matrix(3, 3), m);

        final var bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());
        final var bx2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasXAsAcceleration(bx2);
        assertEquals(bx1, bx2);
        final var by1 = fixer.getBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());
        final var by2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasYAsAcceleration(by2);
        assertEquals(by1, by2);
        final var bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());
        final var bz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasZAsAcceleration(bz2);
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
        final var fixer = new AccelerationFixer();

        // check default value
        final var b1 = fixer.getBias();
        final var b2 = new Matrix(1, 1);
        fixer.getBias(b2);

        assertEquals(new Matrix(3, 1), b1);
        assertEquals(b1, b2);

        // set a new value
        final var b3 = generateBa();
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
        final var fixer = new AccelerationFixer();

        // check default value
        final var b1 = fixer.getBiasArray();
        final var b2 = new double[3];
        fixer.getBiasArray(b2);

        assertArrayEquals(new double[3], b1, 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set a new value
        final var b3 = generateBa().getBuffer();
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
    void testGetSetBiasTriad() {
        final var fixer = new AccelerationFixer();

        // check default value
        final var triad1 = fixer.getBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set a new value
        final var triad2 = new AccelerationTriad();
        triad2.setValueCoordinates(generateBa());
        fixer.setBias(triad2);

        // check
        final var triad3 = fixer.getBiasAsTriad();
        final var triad4 = new AccelerationTriad();
        fixer.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetBiasX() {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasX(), 0.0);

        // set a new value
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        fixer.setBiasX(bx);

        // check
        assertEquals(bx, fixer.getBiasX(), 0.0);
    }

    @Test
    void testGetSetBiasY() {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasY(), 0.0);

        // set a new value
        final var b = generateBa();
        final var by = b.getElementAtIndex(1);
        fixer.setBiasY(by);

        // check
        assertEquals(by, fixer.getBiasY(), 0.0);
    }

    @Test
    void testGetSetBiasZ() {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set a new value
        final var b = generateBa();
        final var bz = b.getElementAtIndex(2);
        fixer.setBiasZ(bz);

        // check
        assertEquals(bz, fixer.getBiasZ(), 0.0);
    }

    @Test
    void testSetBias1() {
        final var fixer = new AccelerationFixer();

        // check default values
        assertEquals(0.0, fixer.getBiasX(), 0.0);
        assertEquals(0.0, fixer.getBiasY(), 0.0);
        assertEquals(0.0, fixer.getBiasZ(), 0.0);

        // set new values
        final var b = generateBa();
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
    void testGetSetBiasXAsAcceleration() {
        final var fixer = new AccelerationFixer();

        // check default value
        final var bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set a new value
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        final var bx2 = new Acceleration(bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasX(bx2);

        // check
        final var bx3 = fixer.getBiasXAsAcceleration();
        final var bx4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasXAsAcceleration(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetBiasYAsAcceleration() {
        final var fixer = new AccelerationFixer();

        // check default value
        final var by1 = fixer.getBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set a new value
        final var b = generateBa();
        final var by = b.getElementAtIndex(1);
        final var by2 = new Acceleration(by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasY(by2);

        // check
        final var by3 = fixer.getBiasYAsAcceleration();
        final var by4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasYAsAcceleration(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetBiasZAsAcceleration() {
        final var fixer = new AccelerationFixer();

        // check default value
        final var bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set a new value
        final var b = generateBa();
        final var bz = b.getElementAtIndex(2);
        final var bz2 = new Acceleration(bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBiasZ(bz2);

        // check
        final var bz3 = fixer.getBiasZAsAcceleration();
        final var bz4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getBiasZAsAcceleration(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testGetSetBias2() {
        final var fixer = new AccelerationFixer();

        // check default values
        final var bx1 = fixer.getBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        final var by1 = fixer.getBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        final var bz1 = fixer.getBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new values
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        final var by = b.getElementAtIndex(1);
        final var bz = b.getElementAtIndex(2);
        final var bx2 = new Acceleration(bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var by2 = new Acceleration(by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bz2 = new Acceleration(bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setBias(bx2, by2, bz2);

        // check
        final var bx3 = fixer.getBiasXAsAcceleration();
        final var by3 = fixer.getBiasYAsAcceleration();
        final var bz3 = fixer.getBiasZAsAcceleration();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    void testGetSetCrossCouplingErrors() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default values
        final var m1 = fixer.getCrossCouplingErrors();
        final var m2 = new Matrix(1, 1);
        fixer.getCrossCouplingErrors(m2);

        assertEquals(new Matrix(3, 3), m1);
        assertEquals(m1, m2);

        // set new values
        final var m3 = generateMa();
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
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getSx(), 0.0);

        // set a new value
        final var m = generateMa();
        final var sx = m.getElementAt(0, 0);

        fixer.setSx(sx);

        // check
        assertEquals(sx, fixer.getSx(), 0.0);
    }

    @Test
    void testGetSetSy() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getSy(), 0.0);

        // set a new value
        final var m = generateMa();
        final var sy = m.getElementAt(1, 1);

        fixer.setSy(sy);

        // check
        assertEquals(sy, fixer.getSy(), 0.0);
    }

    @Test
    void testGetSetSz() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set a new value
        final var m = generateMa();
        final var sz = m.getElementAt(2, 2);

        fixer.setSz(sz);

        // check
        assertEquals(sz, fixer.getSz(), 0.0);
    }

    @Test
    void testGetSetMxy() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMxy(), 0.0);

        // set a new value
        final var m = generateMa();
        final var mxy = m.getElementAt(0, 1);

        fixer.setMxy(mxy);

        // check
        assertEquals(mxy, fixer.getMxy(), 0.0);
    }

    @Test
    void testGetSetMxz() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMxz(), 0.0);

        // set a new value
        final var m = generateMa();
        final var mxz = m.getElementAt(0, 2);

        fixer.setMxz(mxz);

        // check
        assertEquals(mxz, fixer.getMxz(), 0.0);
    }

    @Test
    void testGetSetMyx() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMyx(), 0.0);

        // set a new value
        final var m = generateMa();
        final var myx = m.getElementAt(1, 0);

        fixer.setMyx(myx);

        // check
        assertEquals(myx, fixer.getMyx(), 0.0);
    }

    @Test
    void testGetSetMyz() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMyz(), 0.0);

        // set a new value
        final var m = generateMa();
        final var myz = m.getElementAt(1, 2);

        fixer.setMyz(myz);

        // check
        assertEquals(myz, fixer.getMyz(), 0.0);
    }

    @Test
    void testGetSetMzx() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMzx(), 0.0);

        // set a new value
        final var m = generateMa();
        final var mzx = m.getElementAt(2, 0);

        fixer.setMzx(mzx);

        // check
        assertEquals(mzx, fixer.getMzx(), 0.0);
    }

    @Test
    void testGetSetMzy() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default value
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set a new value
        final var m = generateMa();
        final var mzy = m.getElementAt(2, 1);

        fixer.setMzy(mzy);

        // check
        assertEquals(mzy, fixer.getMzy(), 0.0);
    }

    @Test
    void testSetScalingFactors() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        // check default values
        assertEquals(0.0, fixer.getSx(), 0.0);
        assertEquals(0.0, fixer.getSy(), 0.0);
        assertEquals(0.0, fixer.getSz(), 0.0);

        // set new values
        final var m = generateMa();
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
        final var fixer = new AccelerationFixer();

        // check default values
        assertEquals(0.0, fixer.getMxy(), 0.0);
        assertEquals(0.0, fixer.getMxz(), 0.0);
        assertEquals(0.0, fixer.getMyx(), 0.0);
        assertEquals(0.0, fixer.getMyz(), 0.0);
        assertEquals(0.0, fixer.getMzx(), 0.0);
        assertEquals(0.0, fixer.getMzy(), 0.0);

        // set new values
        final var m = generateMa();
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
        final var fixer = new AccelerationFixer();

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
        final var m = generateMa();
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
    void testFix1() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, new Random());

        final var measuredF = measuredKinematics.getSpecificForceTriad();

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, new double[1]));
    }

    @Test
    void testFix2() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, new Random());

        final var measuredF = measuredKinematics.getSpecificForceTriad();

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredF, result);

        // check
        assertEquals(fx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m1));
        final var m2 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m2));
    }

    @Test
    void testFix3() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.getSpecificForceTriad();

        final var result = new AccelerationTriad();
        fixer.fix(measuredF, result);

        // check
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.getUnit());
        assertEquals(fx, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(fy, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(fz, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testFix4() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getSpecificForceX();
        final var measuredFy = measuredKinematics.getSpecificForceY();
        final var measuredFz = measuredKinematics.getSpecificForceZ();
        final var result = new AccelerationTriad();
        fixer.fix(measuredFx, measuredFy, measuredFz, result);

        // check
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.getUnit());
        assertEquals(fx, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(fy, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(fz, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testFix5() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceArray();

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], result1));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, new double[1]));
    }

    @Test
    void testFix6() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, result1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, result1));
    }

    @Test
    void testFix7() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredF, result1);
        fixer.fix(measuredF, ba, ma, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(fx, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result1.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m2));
    }

    @Test
    void testFix8() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result1 = new double[BodyKinematics.COMPONENTS];
        final var result2 = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredFx, measuredFy, measuredFz, result1);
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma, result2);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);
    }

    @Test
    void testFix9() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result1 = new Matrix(BodyKinematics.COMPONENTS, 1);
        final var result2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredFx, measuredFy, measuredFz, result1);
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma, result2);

        // check
        assertEquals(result1, result2);
        assertEquals(fx, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result1.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz, m2));
    }

    @Test
    void testFix10() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceArray();

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(new double[1], ba, ma, result));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m1, ma, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, m2, ma, result));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, ba, m3, result));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, ba, m4, result));
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, ba, ma, new double[1]));
    }

    @Test
    void testFix11() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m1, ba, ma, result));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(m2, ba, ma, result));
    }

    @Test
    void testFix12() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredF, ba, ma, result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, ba, ma, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredF, ba, ma, m2));
    }

    @Test
    void testFix13() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);
    }

    @Test
    void testFix14() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma, result);

        // check
        assertEquals(fx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result.getElementAtIndex(2), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, ma, m2));
    }

    @Test
    void testFix15() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var result = new double[BodyKinematics.COMPONENTS];
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                result);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);
    }

    @Test
    void testFix16() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var result = new Matrix(BodyKinematics.COMPONENTS, 1);
        fixer.fix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                result);

        // check
        assertEquals(result.getElementAtIndex(0), fx, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(1), fy, ABSOLUTE_ERROR);
        assertEquals(result.getElementAtIndex(2), fz, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fix(measuredFx, measuredFy, measuredFz,
                biasX, biasY, biasZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy, m2));
    }

    @Test
    void testFixAndReturnNew1() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, new Random());

        final var measuredF = measuredKinematics.getSpecificForceTriad();

        final var result = fixer.fixAndReturnNew(measuredF);

        // check
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.getUnit());
        assertEquals(fx, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(fy, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(fz, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew2() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, new Random());

        final var measuredFx = measuredKinematics.getSpecificForceX();
        final var measuredFy = measuredKinematics.getSpecificForceY();
        final var measuredFz = measuredKinematics.getSpecificForceZ();

        final var result = fixer.fixAndReturnNew(measuredFx, measuredFy, measuredFz);

        // check
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, result.getUnit());
        assertEquals(fx, result.getValueX(), ABSOLUTE_ERROR);
        assertEquals(fy, result.getValueY(), ABSOLUTE_ERROR);
        assertEquals(fz, result.getValueZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew3() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceArray();

        final var result1 = fixer.fixAndReturnNew(measuredF);
        final var result2 = fixer.fixAndReturnNew(measuredF, ba, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(new double[1]));
    }

    @Test
    void testFixAndReturnNew4() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result1 = fixer.fixAndReturnNew(measuredF);
        final var result2 = fixer.fixAndReturnNew(measuredF, ba, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m2));
    }

    @Test
    void testFixAndReturnNewMatrix3() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result1 = fixer.fixAndReturnNewMatrix(measuredF);
        final var result2 = fixer.fixAndReturnNewMatrix(measuredF, ba, ma);

        // check
        assertEquals(result1, result2);
        assertEquals(fx, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result1.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew5() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result1 = fixer.fixAndReturnNew(measuredFx, measuredFy, measuredFz);
        final var result2 = fixer.fixAndReturnNew(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma);

        // check
        assertArrayEquals(result1, result2, 0.0);
        assertEquals(fx, result1[0], ABSOLUTE_ERROR);
        assertEquals(fy, result1[1], ABSOLUTE_ERROR);
        assertEquals(fz, result1[2], ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix5() throws AlgebraException {
        final var fixer = new AccelerationFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setBias(ba);
        fixer.setCrossCouplingErrors(ma);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result1 = fixer.fixAndReturnNewMatrix(measuredFx, measuredFy, measuredFz);
        final var result2 = fixer.fixAndReturnNewMatrix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma);

        // check
        assertEquals(result1, result2);
        assertEquals(fx, result1.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result1.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result1.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew6() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceArray();

        final var result = fixer.fixAndReturnNew(measuredF, ba, ma);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(new double[1], ba, ma));
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredF, m1, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredF, m2, ma));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredF, ba, m3));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(measuredF, ba, m4));
    }

    @Test
    void testFixAndReturnNew7() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result = fixer.fixAndReturnNew(measuredF, ba, ma);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m1, ba, ma));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.fixAndReturnNew(m2, ba, ma));
    }

    @Test
    void testFixAndReturnNewMatrix8() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredF = measuredKinematics.asSpecificForceMatrix();

        final var result = fixer.fixAndReturnNewMatrix(measuredF, ba, ma);

        // check
        assertEquals(fx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew8() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result = fixer.fixAndReturnNew(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma);

        // check
        assertEquals(result[0], fx, ABSOLUTE_ERROR);
        assertEquals(result[1], fy, ABSOLUTE_ERROR);
        assertEquals(result[2], fz, ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNewMatrix10() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var result = fixer.fixAndReturnNewMatrix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, ma);

        // check
        assertEquals(fx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result.getElementAtIndex(2), ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew11() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var result = fixer.fixAndReturnNew(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ, sx, sy, sz,
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(fx, result[0], ABSOLUTE_ERROR);
        assertEquals(fy, result[1], ABSOLUTE_ERROR);
        assertEquals(fz, result[2], ABSOLUTE_ERROR);
    }

    @Test
    void testFixAndReturnNew12() throws AlgebraException {
        final var fixer = new AccelerationFixer();

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

        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final var trueKinematics = new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var measuredFx = measuredKinematics.getFx();
        final var measuredFy = measuredKinematics.getFy();
        final var measuredFz = measuredKinematics.getFz();

        final var biasX = ba.getElementAtIndex(0);
        final var biasY = ba.getElementAtIndex(1);
        final var biasZ = ba.getElementAtIndex(2);

        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        final var result = fixer.fixAndReturnNewMatrix(measuredFx, measuredFy, measuredFz, biasX, biasY, biasZ,
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(fx, result.getElementAtIndex(0), ABSOLUTE_ERROR);
        assertEquals(fy, result.getElementAtIndex(1), ABSOLUTE_ERROR);
        assertEquals(fz, result.getElementAtIndex(2), ABSOLUTE_ERROR);
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
