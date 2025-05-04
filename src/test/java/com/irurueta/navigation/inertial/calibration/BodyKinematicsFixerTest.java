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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class BodyKinematicsFixerTest {

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
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(new Matrix(3, 1), fixer.getAccelerationBias());
        final var ba = new Matrix(3, 1);
        fixer.getAccelerationBias(ba);
        assertEquals(new Matrix(3, 1), ba);

        assertArrayEquals(new double[3], fixer.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        fixer.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        assertEquals(0.0, fixer.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, fixer.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, fixer.getAccelerationBiasZ(), 0.0);

        final var accelerationTriad1 = fixer.getAccelerationBiasAsTriad();
        assertEquals(0.0, accelerationTriad1.getValueX(), 0.0);
        assertEquals(0.0, accelerationTriad1.getValueY(), 0.0);
        assertEquals(0.0, accelerationTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerationTriad1.getUnit());
        final var accelerationTriad2 = new AccelerationTriad();
        fixer.getAccelerationBiasAsTriad(accelerationTriad2);
        assertEquals(accelerationTriad1, accelerationTriad2);

        assertEquals(new Matrix(3, 3), fixer.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        fixer.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);

        final var bax1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);
        final var bay1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);
        final var baz1 = fixer.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        assertEquals(0.0, fixer.getAccelerationSx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMxy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMxz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzy(), 0.0);

        assertEquals(new Matrix(3, 1), fixer.getAngularSpeedBias());
        final var bg = new Matrix(3, 1);
        fixer.getAngularSpeedBias(bg);
        assertEquals(new Matrix(3, 1), bg);

        assertArrayEquals(new double[3], fixer.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        fixer.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);

        final var angularSpeedTriad1 = fixer.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, angularSpeedTriad1.getValueX(), 0.0);
        assertEquals(0.0, angularSpeedTriad1.getValueY(), 0.0);
        assertEquals(0.0, angularSpeedTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeedTriad1.getUnit());
        final var angularSpeedTriad2 = new AngularSpeedTriad();
        fixer.getAngularSpeedBiasAsTriad(angularSpeedTriad2);
        assertEquals(angularSpeedTriad1, angularSpeedTriad2);

        assertEquals(0.0, fixer.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);
        final var bgy1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);
        final var bgz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        assertEquals(new Matrix(3, 3), fixer.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        fixer.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);

        assertEquals(0.0, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzy(), 0.0);

        assertEquals(new Matrix(3, 3), fixer.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        fixer.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
    }

    @Test
    void testGetSetAccelerationBias() throws WrongSizeException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var b1 = fixer.getAccelerationBias();
        final var b2 = new Matrix(1, 1);
        fixer.getAccelerationBias(b2);

        assertEquals(new Matrix(3, 1), b1);
        assertEquals(b1, b2);

        // set new value
        final var b3 = generateBa();
        fixer.setAccelerationBias(b3);

        // check
        final var b4 = fixer.getAccelerationBias();
        final var b5 = new Matrix(3, 1);
        fixer.getAccelerationBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAccelerationBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAccelerationBias(m2));
    }

    @Test
    void testGetSetAccelerationBiasArray() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var b1 = fixer.getAccelerationBiasArray();
        final var b2 = new double[3];
        fixer.getAccelerationBiasArray(b2);

        assertArrayEquals(new double[3], b1, 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final var b3 = generateBa().getBuffer();
        fixer.setAccelerationBias(b3);

        // check
        final var b4 = fixer.getAccelerationBiasArray();
        final var b5 = new double[3];
        fixer.getAccelerationBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.getAccelerationBiasArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> fixer.setAccelerationBias(new double[1]));
    }

    @Test
    void testGetSetAccelerationBiasTriad() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var triad1 = fixer.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new value
        final var triad2 = new AccelerationTriad();
        triad2.setValueCoordinates(generateBa());
        fixer.setAccelerationBias(triad2);

        // check
        final var triad3 = fixer.getAccelerationBiasAsTriad();
        final var triad4 = new AccelerationTriad();
        fixer.getAccelerationBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAccelerationBiasX() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationBiasX(), 0.0);

        // set new value
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        fixer.setAccelerationBiasX(bx);

        // check
        assertEquals(bx, fixer.getAccelerationBiasX(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasY() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationBiasY(), 0.0);

        // set new value
        final var b = generateBa();
        final var by = b.getElementAtIndex(1);
        fixer.setAccelerationBiasY(by);

        // check
        assertEquals(by, fixer.getAccelerationBiasY(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasZ() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationBiasZ(), 0.0);

        // set new value
        final var b = generateBa();
        final var bz = b.getElementAtIndex(2);
        fixer.setAccelerationBiasZ(bz);

        // check
        assertEquals(bz, fixer.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testSetAccelerationBias1() {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, fixer.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, fixer.getAccelerationBiasZ(), 0.0);

        // set new values
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        final var by = b.getElementAtIndex(1);
        final var bz = b.getElementAtIndex(2);
        fixer.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getAccelerationBiasX(), 0.0);
        assertEquals(by, fixer.getAccelerationBiasY(), 0.0);
        assertEquals(bz, fixer.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasXAsAcceleration() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var bx1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set new value
        final var b = generateBa();
        final var bx = b.getElementAtIndex(0);
        final var bx2 = new Acceleration(bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasX(bx2);

        // check
        final var bx3 = fixer.getAccelerationBiasXAsAcceleration();
        final var bx4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasXAsAcceleration(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetAccelerationBiasYAsAcceleration() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var by1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set new value
        final var b = generateBa();
        final var by = b.getElementAtIndex(1);
        final var by2 = new Acceleration(by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasY(by2);

        // check
        final var by3 = fixer.getAccelerationBiasYAsAcceleration();
        final var by4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasYAsAcceleration(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetAccelerationBiasZAsAcceleration() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var bz1 = fixer.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new value
        final var b = generateBa();
        final var bz = b.getElementAtIndex(2);
        final var bz2 = new Acceleration(bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        fixer.setAccelerationBiasZ(bz2);

        // check
        final var bz3 = fixer.getAccelerationBiasZAsAcceleration();
        final var bz4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        fixer.getAccelerationBiasZAsAcceleration(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testGetSetBias2() {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        final var bx1 = fixer.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        final var by1 = fixer.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        final var bz1 = fixer.getAccelerationBiasZAsAcceleration();
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
        fixer.setAccelerationBias(bx2, by2, bz2);

        // check
        final var bx3 = fixer.getAccelerationBiasXAsAcceleration();
        final var by3 = fixer.getAccelerationBiasYAsAcceleration();
        final var bz3 = fixer.getAccelerationBiasZAsAcceleration();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    void testGetSetAccelerationCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        final var m1 = fixer.getAccelerationCrossCouplingErrors();
        final var m2 = new Matrix(1, 1);
        fixer.getAccelerationCrossCouplingErrors(m2);

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

        fixer.setAccelerationCrossCouplingErrors(m3);

        // check
        final var m4 = fixer.getAccelerationCrossCouplingErrors();
        final var m5 = new Matrix(3, 3);
        fixer.getAccelerationCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);

        // Force IllegalArgumentException
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAccelerationCrossCouplingErrors(m6));
        final var m7 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAccelerationCrossCouplingErrors(m7));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> fixer.setAccelerationCrossCouplingErrors(wrong));
    }

    @Test
    void testGetSetAccelerationSx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationSx(), 0.0);

        // set new value
        final var m = generateMa();
        final var sx = m.getElementAt(0, 0);

        fixer.setAccelerationSx(sx);

        // check
        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
    }

    @Test
    void testGetSetAccelerationSy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationSy(), 0.0);

        // set new value
        final var m = generateMa();
        final var sy = m.getElementAt(1, 1);

        fixer.setAccelerationSy(sy);

        // check
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
    }

    @Test
    void testGetSetAccelerationSz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationSz(), 0.0);

        // set new value
        final var m = generateMa();
        final var sz = m.getElementAt(2, 2);

        fixer.setAccelerationSz(sz);

        // check
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMxy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMxy(), 0.0);

        // set new value
        final var m = generateMa();
        final var mxy = m.getElementAt(0, 1);

        fixer.setAccelerationMxy(mxy);

        // check
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
    }

    @Test
    void testGetSetAccelerationMxz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMxz(), 0.0);

        // set new value
        final var m = generateMa();
        final var mxz = m.getElementAt(0, 2);

        fixer.setAccelerationMxz(mxz);

        // check
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMyx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMyx(), 0.0);

        // set new value
        final var m = generateMa();
        final var myx = m.getElementAt(1, 0);

        fixer.setAccelerationMyx(myx);

        // check
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
    }

    @Test
    void testGetSetAccelerationMyz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMyz(), 0.0);

        // set new value
        final var m = generateMa();
        final var myz = m.getElementAt(1, 2);

        fixer.setAccelerationMyz(myz);

        // check
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMzx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMzx(), 0.0);

        // set new value
        final var m = generateMa();
        final var mzx = m.getElementAt(2, 0);

        fixer.setAccelerationMzx(mzx);

        // check
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
    }

    @Test
    void testGetSetAccelerationMzy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAccelerationMzy(), 0.0);

        // set new value
        final var m = generateMa();
        final var mzy = m.getElementAt(2, 1);

        fixer.setAccelerationMzy(mzy);

        // check
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    void testSetAccelerationScalingFactors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAccelerationSx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSz(), 0.0);

        // set new values
        final var m = generateMa();
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);

        fixer.setAccelerationScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
    }

    @Test
    void testSetAccelerationCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAccelerationMxy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMxz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzy(), 0.0);

        // set new values
        final var m = generateMa();
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setAccelerationCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    void testSetAccelerationScalingFactorsAndCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAccelerationSx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationSz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMxy(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMxz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMyz(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzx(), 0.0);
        assertEquals(0.0, fixer.getAccelerationMzy(), 0.0);

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

        fixer.setAccelerationScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, fixer.getAccelerationSx(), 0.0);
        assertEquals(sy, fixer.getAccelerationSy(), 0.0);
        assertEquals(sz, fixer.getAccelerationSz(), 0.0);
        assertEquals(mxy, fixer.getAccelerationMxy(), 0.0);
        assertEquals(mxz, fixer.getAccelerationMxz(), 0.0);
        assertEquals(myx, fixer.getAccelerationMyx(), 0.0);
        assertEquals(myz, fixer.getAccelerationMyz(), 0.0);
        assertEquals(mzx, fixer.getAccelerationMzx(), 0.0);
        assertEquals(mzy, fixer.getAccelerationMzy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBias() throws WrongSizeException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var b1 = fixer.getAngularSpeedBias();
        final var b2 = new Matrix(1, 1);
        fixer.getAngularSpeedBias(b2);

        assertEquals(new Matrix(3, 1), b1);
        assertEquals(b1, b2);

        // set new value
        final var b3 = generateBg();
        fixer.setAngularSpeedBias(b3);

        // check
        final var b4 = fixer.getAngularSpeedBias();
        final var b5 = new Matrix(3, 1);
        fixer.getAngularSpeedBias(b5);

        assertEquals(b3, b4);
        assertEquals(b3, b5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedBias(m2));
    }

    @Test
    void testGetSetAngularSpeedBiasArray() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var b1 = fixer.getAngularSpeedBiasArray();
        final var b2 = new double[3];
        fixer.getAngularSpeedBiasArray(b2);

        assertArrayEquals(new double[3], b1, 0.0);
        assertArrayEquals(b1, b2, 0.0);

        // set new value
        final var b3 = generateBg().getBuffer();
        fixer.setAngularSpeedBias(b3);

        // check
        final var b4 = fixer.getAngularSpeedBiasArray();
        final var b5 = new double[3];
        fixer.getAngularSpeedBiasArray(b5);

        assertArrayEquals(b3, b4, 0.0);
        assertArrayEquals(b3, b5, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> fixer.getAngularSpeedBiasArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedBias(new double[1]));
    }

    @Test
    void testGetSetAngularSpeedBiasTriad() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var triad1 = fixer.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final var triad2 = new AngularSpeedTriad();
        triad2.setValueCoordinates(generateBg());
        fixer.setAngularSpeedBias(triad2);

        // check
        final var triad3 = fixer.getAngularSpeedBiasAsTriad();
        final var triad4 = new AngularSpeedTriad();
        fixer.getAngularSpeedBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAngularSpeedBiasX() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedBiasX(), 0.0);

        // set new value
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        fixer.setAngularSpeedBiasX(bx);

        // check
        assertEquals(bx, fixer.getAngularSpeedBiasX(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasY() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedBiasY(), 0.0);

        // set new value
        final var b = generateBg();
        final var by = b.getElementAtIndex(1);
        fixer.setAngularSpeedBiasY(by);

        // check
        assertEquals(by, fixer.getAngularSpeedBiasY(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasZ() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedBiasZ(), 0.0);

        // set new value
        final var b = generateBg();
        final var bz = b.getElementAtIndex(2);
        fixer.setAngularSpeedBiasZ(bz);

        // check
        assertEquals(bz, fixer.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    void testSetAngularSpeedBias1() {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedBiasZ(), 0.0);

        // set new values
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        final var by = b.getElementAtIndex(1);
        final var bz = b.getElementAtIndex(2);
        fixer.setAngularSpeedBias(bx, by, bz);

        // check
        assertEquals(bx, fixer.getAngularSpeedBiasX(), 0.0);
        assertEquals(by, fixer.getAngularSpeedBiasY(), 0.0);
        assertEquals(bz, fixer.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasXAsAngularSpeed() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var bx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        // set new value
        final var b = generateBg();
        final var bx = b.getElementAtIndex(0);
        final var bx2 = new AngularSpeed(bx, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasX(bx2);

        // check
        final var bx3 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        final var bx4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasXAsAngularSpeed(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetAngularSpeedBiasYAsAngularSpeed() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var by1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        // set new value
        final var b = generateBg();
        final var by = b.getElementAtIndex(1);
        final var by2 = new AngularSpeed(by, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasY(by2);

        // check
        final var by3 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        final var by4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasYAsAngularSpeed(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetAngularSpeedBiasZAsAngularSpeed() {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        final var bz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bz1.getUnit());

        // set new value
        final var b = generateBg();
        final var bz = b.getElementAtIndex(2);
        final var bz2 = new AngularSpeed(bz, AngularSpeedUnit.RADIANS_PER_SECOND);
        fixer.setAngularSpeedBiasZ(bz2);

        // check
        final var bz3 = fixer.getAngularSpeedBiasZAsAngularSpeed();
        final var bz4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        fixer.getAngularSpeedBiasZAsAngularSpeed(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testGetSetAngularSpeedBias2() {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        final var bx1 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bx1.getUnit());

        final var by1 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, by1.getUnit());

        final var bz1 = fixer.getAngularSpeedBiasZAsAngularSpeed();
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
        fixer.setAngularSpeedBias(bx2, by2, bz2);

        // check
        final var bx3 = fixer.getAngularSpeedBiasXAsAngularSpeed();
        final var by3 = fixer.getAngularSpeedBiasYAsAngularSpeed();
        final var bz3 = fixer.getAngularSpeedBiasZAsAngularSpeed();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    void testGetSetAngularSpeedCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        final var m1 = fixer.getAngularSpeedCrossCouplingErrors();
        final var m2 = new Matrix(1, 1);
        fixer.getAngularSpeedCrossCouplingErrors(m2);

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

        fixer.setAngularSpeedCrossCouplingErrors(m3);

        // check
        final var m4 = fixer.getAngularSpeedCrossCouplingErrors();
        final var m5 = new Matrix(3, 3);
        fixer.getAngularSpeedCrossCouplingErrors(m5);

        assertEquals(m3, m4);
        assertEquals(m3, m5);

        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);

        // Force IllegalArgumentException
        final var m6 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedCrossCouplingErrors(m6));
        final var m7 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedCrossCouplingErrors(m7));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> fixer.setAngularSpeedCrossCouplingErrors(wrong));
    }

    @Test
    void testGetSetAngularSpeedSx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedSx(), 0.0);

        // set new value
        final var m = generateMg();
        final var sx = m.getElementAt(0, 0);

        fixer.setAngularSpeedSx(sx);

        // check
        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedSy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedSy(), 0.0);

        // set new value
        final var m = generateMg();
        final var sy = m.getElementAt(1, 1);

        fixer.setAngularSpeedSy(sy);

        // check
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedSz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedSz(), 0.0);

        // set new value
        final var m = generateMg();
        final var sz = m.getElementAt(2, 2);

        fixer.setAngularSpeedSz(sz);

        // check
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMxy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMxy(), 0.0);

        // set new value
        final var m = generateMg();
        final var mxy = m.getElementAt(0, 1);

        fixer.setAngularSpeedMxy(mxy);

        // check
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMxz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMxz(), 0.0);

        // set new value
        final var m = generateMg();
        final var mxz = m.getElementAt(0, 2);

        fixer.setAngularSpeedMxz(mxz);

        // check
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMyx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMyx(), 0.0);

        // set new value
        final var m = generateMg();
        final var myx = m.getElementAt(1, 0);

        fixer.setAngularSpeedMyx(myx);

        // check
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMyz() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMyz(), 0.0);

        // set new value
        final var m = generateMg();
        final var myz = m.getElementAt(1, 2);

        fixer.setAngularSpeedMyz(myz);

        // check
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMzx() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMzx(), 0.0);

        // set new value
        final var m = generateMg();
        final var mzx = m.getElementAt(2, 0);

        fixer.setAngularSpeedMzx(mzx);

        // check
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMzy() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default value
        assertEquals(0.0, fixer.getAngularSpeedMzy(), 0.0);

        // set new value
        final var m = generateMg();
        final var mzy = m.getElementAt(2, 1);

        fixer.setAngularSpeedMzy(mzy);

        // check
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testSetAngularSpeedScalingFactors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSz(), 0.0);

        // set new values
        final var m = generateMg();
        final var sx = m.getElementAt(0, 0);
        final var sy = m.getElementAt(1, 1);
        final var sz = m.getElementAt(2, 2);

        fixer.setAngularSpeedScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
    }

    @Test
    void testSetAngularSpeedCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzy(), 0.0);

        // set new values
        final var m = generateMg();
        final var mxy = m.getElementAt(0, 1);
        final var mxz = m.getElementAt(0, 2);
        final var myx = m.getElementAt(1, 0);
        final var myz = m.getElementAt(1, 2);
        final var mzx = m.getElementAt(2, 0);
        final var mzy = m.getElementAt(2, 1);

        fixer.setAngularSpeedCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testSetAngularSpeedScalingFactorsAndCrossCouplingErrors() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        assertEquals(0.0, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, fixer.getAngularSpeedMzy(), 0.0);

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

        fixer.setAngularSpeedScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, fixer.getAngularSpeedSx(), 0.0);
        assertEquals(sy, fixer.getAngularSpeedSy(), 0.0);
        assertEquals(sz, fixer.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, fixer.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, fixer.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, fixer.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, fixer.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, fixer.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, fixer.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedGDependantCrossBias() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        // check default values
        final var g1 = fixer.getAngularSpeedGDependantCrossBias();
        final var g2 = new Matrix(1, 1);
        fixer.getAngularSpeedGDependantCrossBias(g2);

        assertEquals(new Matrix(3, 3), g1);
        assertEquals(g1, g2);

        // set new values
        final var g3 = generateGg();

        fixer.setAngularSpeedGDependantCrossBias(g3);

        // check
        final var g4 = fixer.getAngularSpeedGDependantCrossBias();
        final var g5 = new Matrix(3, 3);
        fixer.getAngularSpeedGDependantCrossBias(g5);

        assertEquals(g3, g4);
        assertEquals(g3, g5);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedGDependantCrossBias(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> fixer.setAngularSpeedGDependantCrossBias(m2));
    }

    @Test
    void testFix1() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var result = new BodyKinematics();
        fixer.fix(measuredKinematics, result);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    void testFix2() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();
        final var trueF = trueKinematics.getSpecificForceTriad();
        final var trueAngularSpeed = trueKinematics.getAngularRateTriad();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());
        final var measuredF = measuredKinematics.getSpecificForceTriad();
        final var measuredAngularSpeed = measuredKinematics.getAngularRateTriad();

        final var fixedF = new AccelerationTriad();
        final var fixedAngularSpeed = new AngularSpeedTriad();
        fixer.fix(measuredF, measuredAngularSpeed, fixedF, fixedAngularSpeed);

        // check
        assertTrue(fixedF.equals(trueF, ABSOLUTE_ERROR));
        assertTrue(fixedAngularSpeed.equals(trueAngularSpeed, ABSOLUTE_ERROR));
    }

    @Test
    void testFix3() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();
        final var trueF = trueKinematics.getSpecificForceTriad();
        final var trueAngularSpeed = trueKinematics.getAngularRateTriad();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var fixedF = new AccelerationTriad();
        final var fixedAngularSpeed = new AngularSpeedTriad();
        fixer.fix(measuredKinematics, fixedF, fixedAngularSpeed);

        // check
        assertTrue(fixedF.equals(trueF, ABSOLUTE_ERROR));
        assertTrue(fixedAngularSpeed.equals(trueAngularSpeed, ABSOLUTE_ERROR));
    }

    @Test
    void testFix4() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());
        final var measuredF = measuredKinematics.getSpecificForceTriad();
        final var measuredAngularSpeed = measuredKinematics.getAngularRateTriad();

        final var result = new BodyKinematics();
        fixer.fix(measuredF, measuredAngularSpeed, result);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew1() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());

        final var result = fixer.fixAndReturnNew(measuredKinematics);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
    }

    @Test
    void testFixAndReturnNew2() throws AlgebraException {
        final var fixer = new BodyKinematicsFixer();

        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMa();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var trueKinematics = getTrueKinematics();

        final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                new Random());
        final var measuredF = measuredKinematics.getSpecificForceTriad();
        final var measuredAngularSpeed = measuredKinematics.getAngularRateTriad();

        final var result = fixer.fixAndReturnNew(measuredF, measuredAngularSpeed);

        // check
        assertTrue(result.equals(trueKinematics, ABSOLUTE_ERROR));
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

    private static BodyKinematics getTrueKinematics() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);
        final var fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE, MAX_ACCELEROMETER_VALUE);

        final var omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final var omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        return new BodyKinematics(fx, fy, fz, omegaX, omegaY, omegaZ);
    }
}
