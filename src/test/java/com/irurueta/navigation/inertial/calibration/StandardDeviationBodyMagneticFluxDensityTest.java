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

import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class StandardDeviationBodyMagneticFluxDensityTest {

    private static final double MIN_MAGNETIC_FLUX_DENSITY = -70e-6;
    private static final double MAX_MAGNETIC_FLUX_DENSITY = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var stdMagnetic = new StandardDeviationBodyMagneticFluxDensity();

        // check default values
        assertNull(stdMagnetic.getMagneticFluxDensity());
        assertEquals(0.0, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density
        final var magneticFluxDensity = new BodyMagneticFluxDensity();
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity);

        // check default values
        assertSame(magneticFluxDensity, stdMagnetic.getMagneticFluxDensity());
        assertEquals(0.0, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with standard deviation
        final var randomizer = new UniformRandomizer();
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(std);

        // check default values
        assertNull(stdMagnetic.getMagneticFluxDensity());
        assertEquals(std, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationBodyMagneticFluxDensity(
                -1.0));

        // test constructor with magnetic flux density and standard deviation
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        // check default values
        assertSame(magneticFluxDensity, stdMagnetic.getMagneticFluxDensity());
        assertEquals(std, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationBodyMagneticFluxDensity(
                magneticFluxDensity, -1.0));

        // test copy constructor
        stdMagnetic = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(
                stdMagnetic);

        // check
        assertEquals(stdMagnetic.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic.getMagneticFluxDensityStandardDeviation(), 
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetMagneticFluxDensity() {
        final var stdMagnetic = new StandardDeviationBodyMagneticFluxDensity();

        // check default value
        assertNull(stdMagnetic.getMagneticFluxDensity());

        // set new value
        final var magneticFluxDensity = new BodyMagneticFluxDensity();
        stdMagnetic.setMagneticFluxDensity(magneticFluxDensity);

        // check
        assertSame(magneticFluxDensity, stdMagnetic.getMagneticFluxDensity());
    }

    @Test
    void testGetSetMagneticFluxDensityStandardDeviation() {
        final var stdMagnetic = new StandardDeviationBodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        stdMagnetic.setMagneticFluxDensityStandardDeviation(std);

        // check
        assertEquals(std, stdMagnetic.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> stdMagnetic.setMagneticFluxDensityStandardDeviation(-1.0));
    }

    @Test
    void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtSourceAndDestinationIsEmpty() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity();

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtDestinationAndSourceIsEmpty() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity();

        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNull(stdMagnetic2.getMagneticFluxDensity());
        assertEquals(0.0, stdMagnetic2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenBodyMagneticFluxDensityIsAvailableAtSourceAndDestination() {
        final var randomizer = new UniformRandomizer();
        final var bx1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std1 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity1 = new BodyMagneticFluxDensity(bx1, by1, bz1);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity1, std1);

        final var bx2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std2 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity2 = new BodyMagneticFluxDensity(bx2, by2, bz2);

        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity2, std2);

        stdMagnetic2.copyFrom(stdMagnetic1);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var bx1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz1 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std1 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity1 = new BodyMagneticFluxDensity(bx1, by1, bz1);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity1, std1);

        final var bx2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz2 = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std2 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity2 = new BodyMagneticFluxDensity(bx2, by2, bz2);

        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity2, std2);

        stdMagnetic1.copyFrom(stdMagnetic2);

        // check
        assertNotSame(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensity(), stdMagnetic2.getMagneticFluxDensity());
        assertEquals(stdMagnetic1.getMagneticFluxDensityStandardDeviation(),
                stdMagnetic2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic3 = new StandardDeviationBodyMagneticFluxDensity();

        assertEquals(stdMagnetic1.hashCode(), stdMagnetic2.hashCode());
        assertNotEquals(stdMagnetic1.hashCode(), stdMagnetic3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic3 = new StandardDeviationBodyMagneticFluxDensity();

        //noinspection EqualsWithItself
        assertEquals(stdMagnetic1, stdMagnetic1);
        assertTrue(stdMagnetic1.equals(stdMagnetic2));
        assertFalse(stdMagnetic1.equals(stdMagnetic3));
        assertNotEquals(null, stdMagnetic1);
        assertNotEquals(new Object(), stdMagnetic1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic2 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);
        final var stdMagnetic3 = new StandardDeviationBodyMagneticFluxDensity();

        assertTrue(stdMagnetic1.equals(stdMagnetic1, THRESHOLD));
        assertTrue(stdMagnetic1.equals(stdMagnetic2, THRESHOLD));
        assertFalse(stdMagnetic1.equals(stdMagnetic3, THRESHOLD));
        assertFalse(stdMagnetic1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        final var stdMagnetic2 = stdMagnetic1.clone();

        // check
        assertEquals(stdMagnetic1, stdMagnetic2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_DENSITY, MAX_MAGNETIC_FLUX_DENSITY);
        final var std = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensity = new BodyMagneticFluxDensity(bx, by, bz);

        final var stdMagnetic1 = new StandardDeviationBodyMagneticFluxDensity(magneticFluxDensity, std);

        final var bytes = SerializationHelper.serialize(stdMagnetic1);

        final var stdMagnetic2 = SerializationHelper.deserialize(bytes);

        assertEquals(stdMagnetic1, stdMagnetic2);
        assertNotSame(stdMagnetic1, stdMagnetic2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = StandardDeviationBodyMagneticFluxDensity.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
