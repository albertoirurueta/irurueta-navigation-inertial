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
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class MagneticFluxDensityTriadTest {

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var triad = new MagneticFluxDensityTriad();

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());
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
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final var vx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final var vy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final var vz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var triad = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());
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
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx1.getUnit());
        final var vx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(vx2);
        assertEquals(0.0, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(0.0, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy1.getUnit());
        final var vy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(vy2);
        assertEquals(0.0, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(0.0, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz1.getUnit());
        final var vz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(0.0, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz2.getUnit());
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new MagneticFluxDensityTriad(valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityTriad.DEFAULT_UNIT, triad.getUnit());
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
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final var vx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final var vy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final var vz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.NANOTESLA, valueX, valueY, valueZ);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());
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
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx1.getUnit());
        final var vx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy1.getUnit());
        final var vy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz1.getUnit());
        final var vz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, vz2.getUnit());
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var bx = new MagneticFluxDensity(valueX, MagneticFluxDensityUnit.TESLA);
        final var by = new MagneticFluxDensity(valueY, MagneticFluxDensityUnit.TESLA);
        final var bz = new MagneticFluxDensity(valueZ, MagneticFluxDensityUnit.TESLA);

        final var values1 = new double[]{valueX, valueY, valueZ};
        final var v1 = Matrix.newFromArray(values1);

        final var triad = new MagneticFluxDensityTriad(bx, by, bz);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityTriad.DEFAULT_UNIT, triad.getUnit());
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
        assertEquals(MagneticFluxDensityUnit.TESLA, vx1.getUnit());
        final var vx2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementX(vx2);
        assertEquals(valueX, vx2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vx2.getUnit());
        final var vy1 = triad.getMeasurementY();
        assertEquals(valueY, vy1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy1.getUnit());
        final var vy2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementY(vy2);
        assertEquals(valueY, vy2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vy2.getUnit());
        final var vz1 = triad.getMeasurementZ();
        assertEquals(valueZ, vz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz1.getUnit());
        final var vz2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementZ(vz2);
        assertEquals(valueZ, vz2.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, vz2.getUnit());
    }

    @Test
    void testConstructor6() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(triad1.getUnit(), triad2.getUnit());
    }

    @Test
    void testGetSetValueX() {
        final var triad = new MagneticFluxDensityTriad();

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
        final var triad = new MagneticFluxDensityTriad();

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
        final var triad = new MagneticFluxDensityTriad();

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
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);

        // set new value
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
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());

        // set new value
        triad.setUnit(MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.setUnit(null));
    }

    @Test
    void testSetValueCoordinatesAndUnit() {
        final var triad = new MagneticFluxDensityTriad();

        // check default values
        assertEquals(0.0, triad.getValueX(), 0.0);
        assertEquals(0.0, triad.getValueY(), 0.0);
        assertEquals(0.0, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ, MagneticFluxDensityUnit.NANOTESLA);

        // check
        assertEquals(valueX, triad.getValueX(), 0.0);
        assertEquals(valueY, triad.getValueY(), 0.0);
        assertEquals(valueZ, triad.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.NANOTESLA, triad.getUnit());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> triad.setValueCoordinatesAndUnit(valueX, valueY, valueZ,
                null));
    }

    @Test
    void testGetSetValuesAsArray() {
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        assertArrayEquals(new double[3], triad.getValuesAsArray(), 0.0);

        // set new value
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
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        assertEquals(new Matrix(3, 1), triad.getValuesAsMatrix());

        // set new value
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
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        final var bx1 = triad.getMeasurementX();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var bx2 = new MagneticFluxDensity(valueX, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementX(bx2);

        // check
        final var bx3 = triad.getMeasurementX();
        final var bx4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementX(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetMeasurementY() {
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        final var by1 = triad.getMeasurementY();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueY = randomizer.nextDouble();
        final var by2 = new MagneticFluxDensity(valueY, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementY(by2);

        // check
        final var by3 = triad.getMeasurementY();
        final var by4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementY(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetMeasurementZ() {
        final var triad = new MagneticFluxDensityTriad();

        // check default value
        final var bz1 = triad.getMeasurementZ();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var valueZ = randomizer.nextDouble();
        final var bz2 = new MagneticFluxDensity(valueZ, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementZ(bz2);

        // check
        final var bz3 = triad.getMeasurementZ();
        final var bz4 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        triad.getMeasurementZ(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testSetMeasurementCoordinates() {
        final var triad = new MagneticFluxDensityTriad();

        // check default values
        final var bx1 = triad.getMeasurementX();
        final var by1 = triad.getMeasurementY();
        final var bz1 = triad.getMeasurementZ();

        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bx1.getUnit());
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, by1.getUnit());
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, bz1.getUnit());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();
        final var bx2 = new MagneticFluxDensity(valueX, MagneticFluxDensityUnit.TESLA);
        final var by2 = new MagneticFluxDensity(valueY, MagneticFluxDensityUnit.TESLA);
        final var bz2 = new MagneticFluxDensity(valueZ, MagneticFluxDensityUnit.TESLA);

        triad.setMeasurementCoordinates(bx2, by2, bz2);

        // check
        final var bx3 = triad.getMeasurementX();
        final var by3 = triad.getMeasurementY();
        final var bz3 = triad.getMeasurementZ();

        assertEquals(bx2, bx3);
        assertEquals(by2, by3);
        assertEquals(bz2, bz3);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.NANOTESLA);

        triad1.copyTo(triad2);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad2.getUnit());
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(MagneticFluxDensityUnit.NANOTESLA);

        triad2.copyFrom(triad1);

        // check
        assertEquals(valueX, triad2.getValueX(), 0.0);
        assertEquals(valueY, triad2.getValueY(), 0.0);
        assertEquals(valueZ, triad2.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad2.getUnit());
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(triad1);
        final var triad3 = new MagneticFluxDensityTriad();

        assertEquals(triad1.hashCode(), triad2.hashCode());
        assertNotEquals(triad1.hashCode(), triad3.hashCode());
    }

    @Test
    void testEquals1() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(triad1);
        final var triad3 = new MagneticFluxDensityTriad();

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

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(triad1);
        final var triad3 = new MagneticFluxDensityTriad();

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

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = new MagneticFluxDensityTriad(triad1);
        final var triad3 = new MagneticFluxDensityTriad();
        final var obj = new Object();

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

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);
        final var triad2 = (MagneticFluxDensityTriad) triad1.clone();

        assertEquals(triad1, triad2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad1 = new MagneticFluxDensityTriad(valueX, valueY, valueZ);

        final var bytes = SerializationHelper.serialize(triad1);
        final var triad2 = SerializationHelper.deserialize(bytes);

        assertEquals(triad1, triad2);
        assertNotSame(triad1, triad2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = MagneticFluxDensityTriad.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    @Test
    void testGetNorm() {
        final var randomizer = new UniformRandomizer();
        final var valueX = randomizer.nextDouble();
        final var valueY = randomizer.nextDouble();
        final var valueZ = randomizer.nextDouble();

        final var triad = new MagneticFluxDensityTriad(valueX, valueY, valueZ);

        final var sqrNorm = valueX * valueX + valueY * valueY + valueZ * valueZ;
        final var norm = Math.sqrt(sqrNorm);

        assertEquals(sqrNorm, triad.getSqrNorm(), 0.0);
        assertEquals(norm, triad.getNorm(), 0.0);

        final var b1 = triad.getMeasurementNorm();
        final var b2 = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        triad.getMeasurementNorm(b2);

        assertEquals(norm, b1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        assertEquals(b1, b2);
    }
}
