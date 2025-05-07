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

import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class TimedBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var timedBodyKinematics = new TimedBodyKinematics();

        // check default values
        assertNull(timedBodyKinematics.getKinematics());
        assertEquals(0.0, timedBodyKinematics.getTimestampSeconds(), 0.0);
        var time1 = timedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        var time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with body kinematics
        final var kinematics = createBodyKinematics();
        timedBodyKinematics = new TimedBodyKinematics(kinematics);

        // check default values
        assertSame(kinematics, timedBodyKinematics.getKinematics());
        assertEquals(0.0, timedBodyKinematics.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with timestamp in seconds
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        timedBodyKinematics = new TimedBodyKinematics(timestampSeconds);

        // check
        assertNull(timedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with timestamp
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);
        timedBodyKinematics = new TimedBodyKinematics(time);

        // check
        assertNull(timedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with body kinematics and timestamp in seconds
        timedBodyKinematics = new TimedBodyKinematics(kinematics, timestampSeconds);

        // check
        assertSame(kinematics, timedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with body kinematics and timestamp
        timedBodyKinematics = new TimedBodyKinematics(kinematics, time);

        // check
        assertSame(kinematics, timedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test copy constructor
        final var timedBodyKinematics2 = new TimedBodyKinematics(timedBodyKinematics);

        // check
        assertEquals(kinematics, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics2.getTimestampSeconds(), 0.0);
        time1 = timedBodyKinematics2.getTimestamp();
        assertEquals(time, time1);
        time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics2.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testGetSetKinematics() {
        final var timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        assertNull(timedBodyKinematics.getKinematics());

        // set a new value
        final var kinematics = new BodyKinematics();
        timedBodyKinematics.setKinematics(kinematics);

        // check
        assertSame(kinematics, timedBodyKinematics.getKinematics());
    }

    @Test
    void testGetSetTimestampSeconds() {
        final var timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        assertEquals(0.0, timedBodyKinematics.getTimestampSeconds(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        timedBodyKinematics.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
    }

    @Test
    void testGetSetTimestamp() {
        final var timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        final var time1 = timedBodyKinematics.getTimestamp();

        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        timedBodyKinematics.setTimestamp(time2);

        final var time3 = timedBodyKinematics.getTimestamp();
        final var time4 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    void testCopyFromWhenBodyKinematicsIsAvailableAtSourceAndDestinationIsEmpty() {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);
        final var timedBodyKinematics2 = new TimedBodyKinematics();

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertEquals(kinematics, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testCopyFromWhenDestinationHasKinematics() {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics();
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics, timestampSeconds);

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertNull(timedBodyKinematics2.getKinematics());
        assertEquals(0.0, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testCopyFromWhenSourceAndDestinationHasKinematics() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertEquals(kinematics1, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        timedBodyKinematics1.copyTo(timedBodyKinematics2);

        // check
        assertEquals(kinematics1, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testHashCode() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        assertEquals(timedBodyKinematics1.hashCode(), timedBodyKinematics2.hashCode());
        assertNotEquals(timedBodyKinematics1.hashCode(), timedBodyKinematics3.hashCode());
    }

    @Test
    void testEquals() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        //noinspection EqualsWithItself
        assertEquals(timedBodyKinematics1, timedBodyKinematics1);
        //noinspection EqualsWithItself
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics1));
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics2));
        assertFalse(timedBodyKinematics1.equals(timedBodyKinematics3));
        // noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertNotEquals(null, timedBodyKinematics1);
        assertFalse(timedBodyKinematics1.equals(null));
        assertNotEquals(new Object(), timedBodyKinematics1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final var timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics1, THRESHOLD));
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics2, THRESHOLD));
        assertFalse(timedBodyKinematics1.equals(timedBodyKinematics3, THRESHOLD));
        assertFalse(timedBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);

        final var timedBodyKinematics2 = timedBodyKinematics1.clone();

        // check
        assertEquals(timedBodyKinematics1, timedBodyKinematics2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);

        final var bytes = SerializationHelper.serialize(timedBodyKinematics1);

        final var timedBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(timedBodyKinematics1, timedBodyKinematics2);
        assertNotSame(timedBodyKinematics1, timedBodyKinematics2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = TimedBodyKinematics.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    private static BodyKinematics createBodyKinematics() {
        final var randomizer = new UniformRandomizer();
        final var fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final var fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final var angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final var angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }
}
