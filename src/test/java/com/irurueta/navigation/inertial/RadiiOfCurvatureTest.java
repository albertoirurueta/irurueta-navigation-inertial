/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class RadiiOfCurvatureTest {

    private static final double MIN_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 - 50.0;
    private static final double MAX_VALUE = Constants.EARTH_EQUATORIAL_RADIUS_WGS84 + 50.0;
    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var radii = new RadiiOfCurvature();

        // check default values
        assertEquals(0.0, radii.getRn(), 0.0);
        assertEquals(0.0, radii.getRe(), 0.0);

        assertEquals(0.0, radii.getRnDistance().getValue().doubleValue(), 0.0);
        assertEquals(0.0, radii.getReDistance().getValue().doubleValue(), 0.0);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        radii = new RadiiOfCurvature(rn, re);

        // check default values
        assertEquals(rn, radii.getRn(), 0.0);
        assertEquals(re, radii.getRe(), 0.0);

        assertEquals(rn, radii.getRnDistance().getValue().doubleValue(), 0.0);
        assertEquals(re, radii.getReDistance().getValue().doubleValue(), 0.0);

        // test constructor with distance values
        final var rnDistance = new Distance(rn, DistanceUnit.METER);
        final var reDistance = new Distance(re, DistanceUnit.METER);

        radii = new RadiiOfCurvature(rnDistance, reDistance);

        // check default values
        assertEquals(rn, radii.getRn(), 0.0);
        assertEquals(re, radii.getRe(), 0.0);

        assertEquals(rn, radii.getRnDistance().getValue().doubleValue(), 0.0);
        assertEquals(re, radii.getReDistance().getValue().doubleValue(), 0.0);

        // test constructor from another instance
        final var radii2 = new RadiiOfCurvature(radii);

        // check default values
        assertEquals(rn, radii2.getRn(), 0.0);
        assertEquals(re, radii2.getRe(), 0.0);
    }

    @Test
    void testGetSetRn() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default value
        assertEquals(0.0, radii.getRn(), 0.0);

        // set a new value
        radii.setRn(rn);

        // check
        assertEquals(rn, radii.getRn(), 0.0);
    }

    @Test
    void testGetSetRe() {
        final var randomizer = new UniformRandomizer();
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default value
        assertEquals(0.0, radii.getRe(), 0.0);

        // set a new value
        radii.setRe(re);

        // check
        assertEquals(re, radii.getRe(), 0.0);
    }

    @Test
    void testSetRadii1() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default values
        assertEquals(0.0, radii.getRn(), 0.0);
        assertEquals(0.0, radii.getRe(), 0.0);

        // set values
        radii.setValues(rn, re);

        // check
        assertEquals(rn, radii.getRn(), 0.0);
        assertEquals(re, radii.getRe(), 0.0);
    }

    @Test
    void testGetSetRnDistance() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default value
        assertEquals(0.0, radii.getRnDistance().getValue().doubleValue(), 0.0);

        // set a new value
        final var rnDistance1 = new Distance(rn, DistanceUnit.METER);
        radii.setRnDistance(rnDistance1);

        // check
        final var rnDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        radii.getRnDistance(rnDistance2);
        final var rnDistance3 = radii.getRnDistance();

        assertEquals(rnDistance1, rnDistance2);
        assertEquals(rnDistance1, rnDistance3);
    }

    @Test
    void testGetSetReDistance() {
        final var randomizer = new UniformRandomizer();
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default value
        assertEquals(0.0, radii.getReDistance().getValue().doubleValue(), 0.0);

        // set a new value
        final var reDistance1 = new Distance(re, DistanceUnit.METER);
        radii.setReDistance(reDistance1);

        // check
        final var reDistance2 = new Distance(0.0, DistanceUnit.KILOMETER);
        radii.getReDistance(reDistance2);
        final var reDistance3 = radii.getReDistance();

        assertEquals(reDistance1, reDistance2);
        assertEquals(reDistance1, reDistance3);
    }

    @Test
    void testSetRadii2() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii = new RadiiOfCurvature();

        // check default values
        assertEquals(0.0, radii.getRnDistance().getValue().doubleValue(), 0.0);
        assertEquals(0.0, radii.getReDistance().getValue().doubleValue(), 0.0);

        // set values
        final var rnDistance = new Distance(rn, DistanceUnit.METER);
        final var reDistance = new Distance(re, DistanceUnit.METER);
        radii.setValues(rnDistance, reDistance);

        // check
        assertEquals(rn, radii.getRn(), 0.0);
        assertEquals(re, radii.getRe(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);
        final var radii2 = new RadiiOfCurvature();

        radii1.copyTo(radii2);

        // check
        assertEquals(radii1.getRn(), radii2.getRn(), 0.0);
        assertEquals(radii1.getRe(), radii2.getRe(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);
        final var radii2 = new RadiiOfCurvature();

        radii2.copyFrom(radii1);

        // check
        assertEquals(radii1.getRn(), radii2.getRn(), 0.0);
        assertEquals(radii1.getRe(), radii2.getRe(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);
        final var radii2 = new RadiiOfCurvature(rn, re);
        final var radii3 = new RadiiOfCurvature();

        assertEquals(radii1.hashCode(), radii2.hashCode());
        assertNotEquals(radii1.hashCode(), radii3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);
        final var radii2 = new RadiiOfCurvature(rn, re);
        final var radii3 = new RadiiOfCurvature();

        //noinspection EqualsWithItself
        assertEquals(radii1, radii1);
        //noinspection EqualsWithItself
        assertTrue(radii1.equals(radii1));
        assertTrue(radii1.equals(radii2));
        assertFalse(radii1.equals(radii3));
        assertNotEquals(null, radii1);
        assertFalse(radii1.equals(null));
        assertNotEquals(new Object(), radii1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);
        final var radii2 = new RadiiOfCurvature(rn, re);
        final var radii3 = new RadiiOfCurvature();

        assertTrue(radii1.equals(radii1, THRESHOLD));
        assertTrue(radii1.equals(radii2, THRESHOLD));
        assertFalse(radii1.equals(radii3, THRESHOLD));
        assertFalse(radii1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);

        final var radii2 = radii1.clone();

        // check
        assertEquals(radii1, radii2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var rn = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var re = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var radii1 = new RadiiOfCurvature(rn, re);

        final var bytes = SerializationHelper.serialize(radii1);
        final var radii2 = SerializationHelper.deserialize(bytes);

        assertEquals(radii1, radii2);
        assertNotSame(radii1, radii2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = RadiiOfCurvature.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
