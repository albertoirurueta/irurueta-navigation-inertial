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

import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class BodyKinematicsAndMagneticFluxDensityTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.
    private static final double MIN_MAGNETIC_FLUX_VALUE = 30e-6;
    private static final double MAX_MAGNETIC_FLUX_VALUE = 70e-6;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor1() {
        final var kb = new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb.getKinematics());
        assertNull(kb.getMagneticFluxDensity());
    }

    @Test
    void testConstructor2() {
        final var kinematics = new BodyKinematics();
        final var kb = new BodyKinematicsAndMagneticFluxDensity(kinematics);

        // check default values
        assertSame(kb.getKinematics(), kinematics);
        assertNull(kb.getMagneticFluxDensity());
    }

    @Test
    void testConstructor3() {
        final var b = new BodyMagneticFluxDensity();
        final var kb = new BodyKinematicsAndMagneticFluxDensity(b);

        // check default values
        assertNull(kb.getKinematics());
        assertSame(b, kb.getMagneticFluxDensity());
    }

    @Test
    void testConstructor4() {
        final var kinematics = new BodyKinematics();
        final var b = new BodyMagneticFluxDensity();
        final var kb = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        // check default values
        assertSame(kb.getKinematics(), kinematics);
        assertSame(b, kb.getMagneticFluxDensity());
    }

    @Test
    void testConstructor5() {
        final var kinematics = createKinematics();
        final var b = createMagneticFlux();
        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kb1);

        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    void testGetSetKinematics() {
        final var kb = new BodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(kb.getKinematics());

        // set a new value
        final var kinematics = new BodyKinematics();
        kb.setKinematics(kinematics);

        // check
        assertSame(kinematics, kb.getKinematics());
    }

    @Test
    void testGetSetMagneticFluxDensity() {
        final var kb = new BodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(kb.getMagneticFluxDensity());

        // set a new value
        final var b = new BodyMagneticFluxDensity();
        kb.setMagneticFluxDensity(b);

        // check
        assertSame(b, kb.getMagneticFluxDensity());
    }

    @Test
    void testCopyFromWhenHasBothKinematicsAndMagneticFlux() {
        final var kinematics = createKinematics();
        final var b = createMagneticFlux();
        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    void testCopyFromWhenHasKinematics() {
        final var kinematics = createKinematics();
        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertEquals(kb1.getKinematics(), kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getKinematics(), kb2.getKinematics());
        assertEquals(kb1, kb2);
    }

    @Test
    void testCopyFromWhenHasMagneticFlux() {
        final var b = createMagneticFlux();
        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(b);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertNull(kb2.getKinematics());
        assertEquals(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertNotSame(kb1.getMagneticFluxDensity(), kb2.getMagneticFluxDensity());
        assertEquals(kb1, kb2);
    }

    @Test
    void testCopyFromWhenNullData() {
        final var kb1 = new BodyKinematicsAndMagneticFluxDensity();

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        // check default values
        assertSame(kinematics, kb2.getKinematics());
        assertSame(b, kb2.getMagneticFluxDensity());

        // copy from
        kb2.copyFrom(kb1);

        // check
        assertNull(kb2.getKinematics());
        assertNull(kb2.getMagneticFluxDensity());
    }

    @Test
    void testCopyTo() {
        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        kb1.copyTo(kb2);

        // check
        assertEquals(kinematics1, kb1.getKinematics());
        assertEquals(b1, kb1.getMagneticFluxDensity());
    }

    @Test
    void testHashCode() {
        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb3 = new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        assertEquals(kb1.hashCode(), kb2.hashCode());
        assertNotEquals(kb1.hashCode(), kb3.hashCode());
    }

    @Test
    void testEquals() {
        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb3 = new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        //noinspection EqualsWithItself
        assertEquals(kb1, kb1);
        //noinspection EqualsWithItself
        assertTrue(kb1.equals(kb1));
        assertTrue(kb1.equals(kb2));
        assertFalse(kb1.equals(kb3));
        assertNotEquals(null, kb1);
        assertFalse(kb1.equals(null));
        assertNotEquals(new Object(), kb1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb2 = new BodyKinematicsAndMagneticFluxDensity(kinematics1, b1);

        final var kb3 = new BodyKinematicsAndMagneticFluxDensity(kinematics2, b2);

        assertTrue(kb1.equals(kb1, THRESHOLD));
        assertTrue(kb1.equals(kb2, THRESHOLD));
        assertFalse(kb1.equals(kb3, THRESHOLD));
        assertFalse(kb1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var kinematics = createKinematics();
        final var b = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final Object kb2 = kb1.clone();

        // check
        assertEquals(kb1, kb2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var kinematics = createKinematics();
        final var b = createMagneticFlux();

        final var kb1 = new BodyKinematicsAndMagneticFluxDensity(kinematics, b);

        final byte[] bytes = SerializationHelper.serialize(kb1);
        final var kb2 = SerializationHelper.deserialize(bytes);

        assertEquals(kb1, kb2);
        assertNotSame(kb1, kb2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = BodyKinematicsAndMagneticFluxDensity.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    private static BodyKinematics createKinematics() {
        final var randomizer = new UniformRandomizer();

        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }

    private static BodyMagneticFluxDensity createMagneticFlux() {
        final var randomizer = new UniformRandomizer();

        final var bx = randomizer.nextDouble(MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final var by = randomizer.nextDouble(MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);
        final var bz = randomizer.nextDouble(MIN_MAGNETIC_FLUX_VALUE, MAX_MAGNETIC_FLUX_VALUE);

        return new BodyMagneticFluxDensity(bx, by, bz);
    }
}
