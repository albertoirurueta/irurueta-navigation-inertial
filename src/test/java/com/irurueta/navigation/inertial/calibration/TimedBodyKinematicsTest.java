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
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class TimedBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        TimedBodyKinematics timedBodyKinematics = new TimedBodyKinematics();

        // check default values
        assertNull(timedBodyKinematics.getKinematics());
        assertEquals(0.0, timedBodyKinematics.getTimestampSeconds(), 0.0);
        Time time1 = timedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        Time time2 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // test constructor with body kinematics
        final BodyKinematics kinematics = createBodyKinematics();
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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

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
        final Time time = new Time(timestampSeconds, TimeUnit.SECOND);
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
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(timedBodyKinematics);

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
    public void testGetSetKinematics() {
        final TimedBodyKinematics timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        assertNull(timedBodyKinematics.getKinematics());

        // set new value
        final BodyKinematics kinematics = new BodyKinematics();
        timedBodyKinematics.setKinematics(kinematics);

        // check
        assertSame(kinematics, timedBodyKinematics.getKinematics());
    }

    @Test
    public void testGetSetTimestampSeconds() {
        final TimedBodyKinematics timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        assertEquals(0.0, timedBodyKinematics.getTimestampSeconds(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        timedBodyKinematics.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(timestampSeconds, timedBodyKinematics.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testGetSetTimestamp() {
        final TimedBodyKinematics timedBodyKinematics = new TimedBodyKinematics();

        // check default value
        final Time time1 = timedBodyKinematics.getTimestamp();

        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final Time time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        timedBodyKinematics.setTimestamp(time2);

        final Time time3 = timedBodyKinematics.getTimestamp();
        final Time time4 = new Time(0.0, TimeUnit.HOUR);
        timedBodyKinematics.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    public void testCopyFromWhenBodyKinematicsIsAvailableAtSourceAndDestinationIsEmpty() {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics();

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertEquals(kinematics, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testCopyFromWhenDestinationHasKinematics() {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics();
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics, timestampSeconds);

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertNull(timedBodyKinematics2.getKinematics());
        assertEquals(0.0, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testCopyFromWhenSourceAndDestinationHasKinematics() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        timedBodyKinematics2.copyFrom(timedBodyKinematics1);

        // check
        assertEquals(kinematics1, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        timedBodyKinematics1.copyTo(timedBodyKinematics2);

        // check
        assertEquals(kinematics1, timedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, timedBodyKinematics2.getTimestampSeconds(), 0.0);
    }

    @Test
    public void testHashCode() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        assertEquals(timedBodyKinematics1.hashCode(), timedBodyKinematics2.hashCode());
        assertNotEquals(timedBodyKinematics1.hashCode(), timedBodyKinematics3.hashCode());
    }

    @Test
    public void testEquals() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        //noinspection EqualsWithItself
        assertEquals(timedBodyKinematics1, timedBodyKinematics1);
        //noinspection EqualsWithItself
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics1));
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics2));
        assertFalse(timedBodyKinematics1.equals(timedBodyKinematics3));
        // noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertNotEquals(timedBodyKinematics1, null);
        assertFalse(timedBodyKinematics1.equals(null));
        assertNotEquals(timedBodyKinematics1, new Object());
    }

    @Test
    public void testEqualsWithThreshold() {
        final BodyKinematics kinematics1 = createBodyKinematics();
        final BodyKinematics kinematics2 = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final double timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics2 = new TimedBodyKinematics(kinematics1, timestampSeconds1);
        final TimedBodyKinematics timedBodyKinematics3 = new TimedBodyKinematics(kinematics2, timestampSeconds2);

        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics1, THRESHOLD));
        assertTrue(timedBodyKinematics1.equals(timedBodyKinematics2, THRESHOLD));
        assertFalse(timedBodyKinematics1.equals(timedBodyKinematics3, THRESHOLD));
        assertFalse(timedBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);

        final Object timedBodyKinematics2 = timedBodyKinematics1.clone();

        // check
        assertEquals(timedBodyKinematics1, timedBodyKinematics2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final BodyKinematics kinematics = createBodyKinematics();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final TimedBodyKinematics timedBodyKinematics1 = new TimedBodyKinematics(kinematics, timestampSeconds);

        final byte[] bytes = SerializationHelper.serialize(timedBodyKinematics1);

        final TimedBodyKinematics timedBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(timedBodyKinematics1, timedBodyKinematics2);
        assertNotSame(timedBodyKinematics1, timedBodyKinematics2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = TimedBodyKinematics.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    private static BodyKinematics createBodyKinematics() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateX = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final double angularRateY = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);
        final double angularRateZ = randomizer.nextDouble(MIN_ANGULAR_RATE_VALUE, MAX_ANGULAR_RATE_VALUE);

        return new BodyKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
    }
}
