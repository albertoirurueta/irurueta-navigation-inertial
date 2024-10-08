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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class AngularSpeedTriadTest {

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedTriad.DEFAULT_UNIT, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final double[] values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final Matrix v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final AngularSpeed vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx1.getUnit());
        final AngularSpeed vx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx2.getUnit());
        final AngularSpeed vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy1.getUnit());
        final AngularSpeed vy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy2.getUnit());
        final AngularSpeed vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz1.getUnit());
        final AngularSpeed vz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz2.getUnit());
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final AngularSpeedTriad triad = new AngularSpeedTriad(AngularSpeedUnit.DEGREES_PER_SECOND);

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);
        final double[] values = new double[3];
        triad.getValuesAsArray(values);
        assertArrayEquals(new double[3], values, 0.0);
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());
        final Matrix v = new Matrix(3, 1);
        triad.getValuesAsMatrix(v);
        assertEquals(new Matrix(3, 1), v);
        final AngularSpeed vx1 = triad.getMeasurementX();
        assertEquals(0.0, vx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vx1.getUnit());
        final AngularSpeed vx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vx2.getUnit());
        final AngularSpeed vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vy1.getUnit());
        final AngularSpeed vy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vy2.getUnit());
        final AngularSpeed vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vz1.getUnit());
        final AngularSpeed vz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vz2.getUnit());
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final double[] values1 = new double[]{valueX, valueY, valueZ};
        final Matrix v1 = Matrix.newFromArray(values1);

        final AngularSpeedTriad triad = new AngularSpeedTriad(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedTriad.DEFAULT_UNIT, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final AngularSpeed vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx1.getUnit());
        final AngularSpeed vx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx2.getUnit());
        final AngularSpeed vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy1.getUnit());
        final AngularSpeed vy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy2.getUnit());
        final AngularSpeed vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz1.getUnit());
        final AngularSpeed vz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz2.getUnit());
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final double[] values1 = new double[]{valueX, valueY, valueZ};
        final Matrix v1 = Matrix.newFromArray(values1);

        final AngularSpeedTriad triad = new AngularSpeedTriad(AngularSpeedUnit.DEGREES_PER_SECOND,
                valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final AngularSpeed vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vx1.getUnit());
        final AngularSpeed vx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vx2.getUnit());
        final AngularSpeed vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vy1.getUnit());
        final AngularSpeed vy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vy2.getUnit());
        final AngularSpeed vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vz1.getUnit());
        final AngularSpeed vz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, vz2.getUnit());
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeed mx = new AngularSpeed(valueX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed my = new AngularSpeed(valueY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed mz = new AngularSpeed(valueZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        final double[] values1 = new double[]{valueX, valueY, valueZ};
        final Matrix v1 = Matrix.newFromArray(values1);

        final AngularSpeedTriad triad = new AngularSpeedTriad(mx, my, mz);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.getUnit());
        assertArrayEquals(new double[]{valueX, valueY, valueZ}, triad.getValuesAsArray(), 0.0);
        final double[] values2 = new double[3];
        triad.getValuesAsArray(values2);
        assertArrayEquals(values1, values2, 0.0);
        assertEquals(v1, triad.getValuesAsMatrix());
        final Matrix v2 = new Matrix(3, 1);
        triad.getValuesAsMatrix(v2);
        assertEquals(v1, v2);
        final AngularSpeed vx1 = triad.getMeasurementX();
        assertEquals(valueX, vx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx1.getUnit());
        final AngularSpeed vx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vx2.getUnit());
        final AngularSpeed vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy1.getUnit());
        final AngularSpeed vy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vy2.getUnit());
        final AngularSpeed vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz1.getUnit());
        final AngularSpeed vz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, vz2.getUnit());
    }

    @Test
    public void testConstructor6() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(triad1.getUnit(), triad2.getUnit());
    }

    @Test
    public void testGetSetValueX() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();

        triad.setValueX(valueX);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
    }

    @Test
    public void testGetSetValueY() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueY = randomizer.nextDouble();

        triad.setValueY(valueY);

        // check
        assertEquals(valueY, triad.getValueY(), 0.0);
    }

    @Test
    public void testGetSetValueZ() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueZ = randomizer.nextDouble();

        triad.setValueZ(valueZ);

        // check
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    public void testSetValueCoordinates() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        triad.setValueCoordinates(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
    }

    @Test
    public void testGetSetUnit() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.getUnit());

        // set new value
        triad.setUnit(AngularSpeedUnit.DEGREES_PER_SECOND);

        // check
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.setUnit(null));
    }

    @Test
    public void testSetValueCoordinatesAndUnit() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default values
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, AngularSpeedUnit.DEGREES_PER_SECOND);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.DEGREES_PER_SECOND, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, null));
    }

    @Test
    public void testGetSetValuesAsArray() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] values1 = new double[3];
        randomizer.fill(values1);

        triad.setValueCoordinates(values1);

        // check
        final double[] values2 = triad.getValuesAsArray();
        final double[] values3 = new double[3];
        triad.getValuesAsArray(values3);

        assertArrayEquals(values1, values2, 0.0);
        assertArrayEquals(values1, values3, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.getValuesAsArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> triad.setValueCoordinates(new double[2]));
    }

    @Test
    public void testGetSetValuesAsMatrix() throws WrongSizeException {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());

        // set new value
        final Matrix values1 = Matrix.createWithUniformRandomValues(3, 1, -1.0, 1.0);

        triad.setValueCoordinates(values1);

        // check
        final Matrix values2 = triad.getValuesAsMatrix();
        final Matrix values3 = new Matrix(3, 1);
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
    public void testGetSetMeasurementX() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        final AngularSpeed mx1 = triad.getMeasurementX();
        assertEquals(0.0, mx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, mx1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final AngularSpeed mx2 = new AngularSpeed(valueX, AngularSpeedUnit.RADIANS_PER_SECOND);

        triad.setMeasurementX(mx2);

        // check
        final AngularSpeed mx3 = triad.getMeasurementX();
        final AngularSpeed mx4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementX(mx4);

        assertEquals(mx2, mx3);
        assertEquals(mx2, mx4);
    }

    @Test
    public void testGetSetMeasurementY() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        final AngularSpeed my1 = triad.getMeasurementY();
        assertEquals(0.0, my1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, my1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueY = randomizer.nextDouble();
        final AngularSpeed my2 = new AngularSpeed(valueY, AngularSpeedUnit.RADIANS_PER_SECOND);

        triad.setMeasurementY(my2);

        // check
        final AngularSpeed my3 = triad.getMeasurementY();
        final AngularSpeed my4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementY(my4);

        assertEquals(my2, my3);
        assertEquals(my2, my4);
    }

    @Test
    public void testGetSetMeasurementZ() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default value
        final AngularSpeed mz1 = triad.getMeasurementZ();
        assertEquals(0.0, mz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, mz1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueZ = randomizer.nextDouble();
        final AngularSpeed mz2 = new AngularSpeed(valueZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        triad.setMeasurementZ(mz2);

        // check
        final AngularSpeed mz3 = triad.getMeasurementZ();
        final AngularSpeed mz4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementZ(mz4);

        assertEquals(mz2, mz3);
        assertEquals(mz2, mz4);
    }

    @Test
    public void testSetMeasurementCoordinates() {
        final AngularSpeedTriad triad = new AngularSpeedTriad();

        // check default values
        final AngularSpeed mx1 = triad.getMeasurementX();
        final AngularSpeed my1 = triad.getMeasurementY();
        final AngularSpeed mz1 = triad.getMeasurementZ();

        assertEquals(0.0, mx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, mx1.getUnit());
        assertEquals(0.0, my1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, my1.getUnit());
        assertEquals(0.0, mz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, mz1.getUnit());

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();
        final AngularSpeed mx2 = new AngularSpeed(valueX, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed my2 = new AngularSpeed(valueY, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed mz2 = new AngularSpeed(valueZ, AngularSpeedUnit.RADIANS_PER_SECOND);

        triad.setMeasurementCoordinates(mx2, my2, mz2);

        // check
        final AngularSpeed mx3 = triad.getMeasurementX();
        final AngularSpeed my3 = triad.getMeasurementY();
        final AngularSpeed mz3 = triad.getMeasurementZ();

        assertEquals(mx2, mx3);
        assertEquals(my2, my3);
        assertEquals(mz2, mz3);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(AngularSpeedUnit.DEGREES_PER_SECOND);

        triad1.copyTo(triad2);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(AngularSpeedUnit.DEGREES_PER_SECOND);

        triad2.copyFrom(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad2.getUnit());
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(triad1);
        final AngularSpeedTriad triad3 = new AngularSpeedTriad();

        assertEquals(triad1.hashCode(), triad2.hashCode());
        assertNotEquals(triad1.hashCode(), triad3.hashCode());
    }

    @Test
    public void testEquals1() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(triad1);
        final AngularSpeedTriad triad3 = new AngularSpeedTriad();

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
    public void testEquals2() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(triad1);
        final AngularSpeedTriad triad3 = new AngularSpeedTriad();

        assertTrue(triad1.equals(triad2, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad1.equals(triad1, ABSOLUTE_ERROR));
        assertTrue(triad2.equals(triad2, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad2.equals(triad3, ABSOLUTE_ERROR));
        assertFalse(triad1.equals(null, ABSOLUTE_ERROR));
    }

    @Test
    public void testEquals3() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(triad1);
        final AngularSpeedTriad triad3 = new AngularSpeedTriad();
        final Object obj = new Object();

        assertEquals(triad1, triad2);
        assertNotEquals(triad1, triad3);
        assertNotEquals(triad1, obj);
        assertNotEquals(triad1, null);
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);
        final AngularSpeedTriad triad2 = (AngularSpeedTriad) triad1.clone();

        assertEquals(triad1, triad2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad1 = new AngularSpeedTriad(valueX, valueY, valueZ);

        final byte[] bytes = SerializationHelper.serialize(triad1);
        final AngularSpeedTriad triad2 = SerializationHelper.deserialize(bytes);

        assertEquals(triad1, triad2);
        assertNotSame(triad1, triad2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = AngularSpeedTriad.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    @Test
    public void testGetNorm() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueX = randomizer.nextDouble();
        final double valueY = randomizer.nextDouble();
        final double valueZ = randomizer.nextDouble();

        final AngularSpeedTriad triad = new AngularSpeedTriad(valueX, valueY, valueZ);

        final double sqrNorm = valueX * valueX + valueY * valueY + valueZ * valueZ;
        final double norm = Math.sqrt(sqrNorm);

        assertEquals(sqrNorm, triad.getSqrNorm(), 0.0);
        assertEquals(norm, triad.getNorm(), 0.0);

        final AngularSpeed w1 = triad.getMeasurementNorm();
        final AngularSpeed w2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        triad.getMeasurementNorm(w2);

        assertEquals(norm, w1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        assertEquals(w1, w2);
    }
}
