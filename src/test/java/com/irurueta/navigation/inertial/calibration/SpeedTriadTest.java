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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class SpeedTriadTest {

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var triad = new SpeedTriad();

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final var values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final var v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final var vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.getUnit());
        final var vx2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        final var vy2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.getUnit());
        final var vz2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.getUnit());
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var triad = new SpeedTriad(SpeedUnit.FEET_PER_SECOND);

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final var values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final var v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final var vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vx1.getUnit());
        final var vx2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        final var vy2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vz1.getUnit());
        final var vz2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vz2.getUnit());
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new SpeedTriad(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final var values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final var v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final var vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.getUnit());
        final var vx2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        final var vy2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.getUnit());
        final var vz2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.getUnit());
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new SpeedTriad(SpeedUnit.FEET_PER_SECOND, valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final var values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final var v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final var vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vx1.getUnit());
        final var vx2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        final var vy2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vz1.getUnit());
        final var vz2 = new Speed(1.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, vz2.getUnit());
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var mx = new Speed(valueX, SpeedUnit.METERS_PER_SECOND);
        final var my = new Speed(valueY, SpeedUnit.METERS_PER_SECOND);
        final var mz = new Speed(valueZ, SpeedUnit.METERS_PER_SECOND);

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new SpeedTriad(mx, my, mz);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(SpeedTriad.DEFAULT_UNIT, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final var values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final var v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final var vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx1.getUnit());
        final var vx2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        final var vy2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz1.getUnit());
        final var vz2 = new Speed(1.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, vz2.getUnit());
    }

    @Test
    void testConstructor6() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(triad1);

        // check
        assertEquals(triad1.getValueX(), triad2.getValueX(), 0.0);
        assertEquals(triad1.getValueY(), triad2.getValueY(), 0.0);
        assertEquals(triad1.getValueZ(), triad2.getValueZ(), 0.0);
        assertEquals(triad1.getUnit(), triad2.getUnit());
    }

    @Test
    void testGetSetValueX() {
        final var triad = new SpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();

        triad.setValueX(valueX);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
    }

    @Test
    void testGetSetValueY() {
        final var triad = new SpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueY(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueY = randomizer.nextDouble();

        triad.setValueY(valueY);

        // check
        assertEquals(valueY, triad.getValueY(), 0.0);
    }

    @Test
    void testGetSetValueZ() {
        final var triad = new SpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueZ = randomizer.nextDouble();

        triad.setValueZ(valueZ);

        // check
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    void testSetValueCoordinates() {
        final var triad = new SpeedTriad();

        // check default values
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        triad.setValueCoordinates(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    void testGetSetUnit() {
        final var triad = new SpeedTriad();

        // check default value
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.getUnit());

        // set new value
        triad.setUnit(SpeedUnit.KILOMETERS_PER_HOUR);

        // check
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.setUnit(null));
    }

    @Test
    void testSetValueCoordinatesAndUnit() {
        final var triad = new SpeedTriad();

        // check default values
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, SpeedUnit.KILOMETERS_PER_HOUR);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(SpeedUnit.KILOMETERS_PER_HOUR, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () ->
                triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, null));
    }

    @Test
    void testGetSetValuesAsArray() {
        final var triad = new SpeedTriad();

        // check default values
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var values1 = new double[3];
        randomizer.fill(values1);

        triad.setValueCoordinates(values1);

        // check
        final var values2 = triad.getValuesAsArray();
        final var values3 = new double[3];
        triad.getValuesAsArray(values3);

        assertArrayEquals(values1, values2, 0.0);
        assertArrayEquals(values1, values3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.getValuesAsArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> triad.setValueCoordinates(new double[2]));
    }

    @Test
    void testGetSetValuesAsMatrix() throws WrongSizeException {
        final var triad = new SpeedTriad();

        // check default values
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());

        // set new values
        final var values1 = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        triad.setValueCoordinates(values1);

        // check
        final var values2 = triad.getValuesAsMatrix();
        final var values3 = new Matrix(3, 1);
        triad.getValuesAsMatrix(values3);

        assertEquals(values1, values2);
        assertEquals(values1, values3);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> triad.getValuesAsMatrix(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> triad.getValuesAsMatrix(m2));
        final var m3 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> triad.setValueCoordinates(m3));
        final var m4 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> triad.setValueCoordinates(m4));
    }

    @Test
    void testGetSetMeasurementX() {
        final var triad = new SpeedTriad();

        // check default value
        final var mx1 = triad.getMeasurementX();
        assertEquals(0.0, mx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, mx1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var mx2 = new Speed(valueX, SpeedUnit.METERS_PER_SECOND);

        triad.setMeasurementX(mx2);

        // check
        final var mx3 = triad.getMeasurementX();
        final var mx4 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementX(mx4);

        assertEquals(mx2, mx3);
        assertEquals(mx2, mx4);
    }

    @Test
    void testGetSetMeasurementY() {
        final var triad = new SpeedTriad();

        // check default value
        final var my1 = triad.getMeasurementY();
        assertEquals(0.0, my1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, my1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueY = randomizer.nextDouble();
        final var my2 = new Speed(valueY, SpeedUnit.METERS_PER_SECOND);

        triad.setMeasurementY(my2);

        // check
        final var my3 = triad.getMeasurementY();
        final var my4 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementY(my4);

        assertEquals(my2, my3);
        assertEquals(my2, my4);
    }

    @Test
    void testGetSetMeasurementZ() {
        final var triad = new SpeedTriad();

        // check default value
        final var mz1 = triad.getMeasurementZ();
        assertEquals(0.0, mz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, mz1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueZ = randomizer.nextDouble();
        final var mz2 = new Speed(valueZ, SpeedUnit.METERS_PER_SECOND);

        triad.setMeasurementZ(mz2);

        // check
        final var mz3 = triad.getMeasurementZ();
        final var mz4 = new Speed(0.0, SpeedUnit.METERS_PER_SECOND);
        triad.getMeasurementZ(mz4);

        assertEquals(mz2, mz3);
        assertEquals(mz2, mz4);
    }

    @Test
    void testSetMeasurementCoordinates() {
        final var triad = new SpeedTriad();

        // check default values
        final var mx1 = triad.getMeasurementX();
        final var my1 = triad.getMeasurementY();
        final var mz1 = triad.getMeasurementZ();

        assertEquals(0.0, mx1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, mx1.getUnit());
        assertEquals(0.0, my1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, my1.getUnit());
        assertEquals(0.0, mz1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, mz1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        final var mx2 = new Speed(valueX, SpeedUnit.METERS_PER_SECOND);
        final var my2 = new Speed(valueY, SpeedUnit.METERS_PER_SECOND);
        final var mz2 = new Speed(valueZ, SpeedUnit.METERS_PER_SECOND);

        triad.setMeasurementCoordinates(mx2, my2, mz2);

        // check
        final var mx3 = triad.getMeasurementX();
        final var my3 = triad.getMeasurementY();
        final var mz3 = triad.getMeasurementZ();

        assertEquals(mx2, mx3);
        assertEquals(my2, my3);
        assertEquals(mz2, mz3);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(SpeedUnit.FEET_PER_SECOND);

        triad1.copyTo(triad2);

        // check
        assertEquals(triad1.getValueX(), triad2.getValueX(), 0.0);
        assertEquals(triad1.getValueY(), triad2.getValueY(), 0.0);
        assertEquals(triad1.getValueZ(), triad2.getValueZ(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad2.getUnit());
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(SpeedUnit.FEET_PER_SECOND);

        triad2.copyFrom(triad1);

        // check
        assertEquals(triad1.getValueX(), triad2.getValueX(), 0.0);
        assertEquals(triad1.getValueY(), triad2.getValueY(), 0.0);
        assertEquals(triad1.getValueZ(), triad2.getValueZ(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, triad2.getUnit());
    }

    @Test
    void testCopy() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(SpeedUnit.FEET_PER_SECOND, valueX, valueY, valueZ);
        final var triad2 = triad1.copy();

        // check
        assertEquals(triad1.getValueX(), triad2.getValueX(), 0.0);
        assertEquals(triad1.getValueY(), triad2.getValueY(), 0.0);
        assertEquals(triad1.getValueZ(), triad2.getValueZ(), 0.0);
        assertEquals(SpeedUnit.FEET_PER_SECOND, triad2.getUnit());
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(triad1);
        final var triad3 = new SpeedTriad();

        assertEquals(triad1.hashCode(), triad2.hashCode());
        assertNotEquals(triad1.hashCode(), triad3.hashCode());
    }

    @Test
    void testEquals1() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(triad1);
        final var triad3 = new SpeedTriad();

        assertTrue(triad1.equals(triad2));
        assertTrue(triad2.equals(triad1));
        //noinspection EqualsWithItself
        assertTrue(triad1.equals(triad1));
        //noinspection EqualsWithItself
        assertTrue(triad2.equals(triad2));
        assertFalse(triad1.equals(triad3));
        assertFalse(triad2.equals(triad3));
        assertFalse(triad1.equals(null));
    }

    @Test
    void testEquals2() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(triad1);
        final var triad3 = new SpeedTriad();

        assertTrue(triad1.equals(triad2, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad1.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad2, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad2.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(null, ABSOLUTE_ERROR));
    }

    @Test
    void testEquals3() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = new SpeedTriad(triad1);
        final var triad3 = new SpeedTriad();
        final Object obj = new Object();

        assertEquals(triad1, triad2);
        assertNotEquals(triad1, triad3);
        assertNotEquals(triad1, obj);
        assertNotEquals(null, triad1);
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);
        final var triad2 = (SpeedTriad) triad1.clone();

        assertEquals(triad1, triad2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new SpeedTriad(valueX, valueY, valueZ);

        final var bytes = SerializationHelper.serialize(triad1);
        final var triad2 = SerializationHelper.deserialize(bytes);

        assertEquals(triad1, triad2);
        assertNotSame(triad1, triad2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = SpeedTriad.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    @Test
    void testGetNorm() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad = new SpeedTriad(valueX, valueY, valueZ);

        final var sqrNorm = valueX * valueX + valueY * valueY + valueZ * valueZ;
        final var norm = Math.sqrt(sqrNorm);

        assertEquals(sqrNorm, triad.getSqrNorm(), 0.0);
        assertEquals(norm, triad.getNorm(), 0.0);

        final var a1 = triad.getMeasurementNorm();
        final var a2 = new Speed(0.0, SpeedUnit.FEET_PER_SECOND);
        triad.getMeasurementNorm(a2);

        assertEquals(norm, a1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, a1.getUnit());
        assertEquals(a1, a2);
    }
}