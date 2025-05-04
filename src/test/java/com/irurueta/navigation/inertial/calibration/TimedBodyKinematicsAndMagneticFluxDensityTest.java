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
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class TimedBodyKinematicsAndMagneticFluxDensityTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    // Typical minimum and minimum magnitude of magnetic flux density
    // at Earth's surface.
    private static final double MIN_MAGNETIC_FLUX_VALUE = 30e-6;
    private static final double MAX_MAGNETIC_FLUX_VALUE = 70e-6;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor1() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(0.0, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(0.0, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor2() {
        final var kinematics = new BodyKinematics();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics);

        // check default values
        assertSame(tkb.getKinematics(), kinematics);
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(0.0, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(0.0, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor3() {
        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(tkb.getMagneticFluxDensity(), b);
        assertEquals(0.0, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(0.0, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor4() {
        final var kinematics = new BodyKinematics();
        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b);

        // check default values
        assertSame(kinematics, tkb.getKinematics());
        assertSame(b, tkb.getMagneticFluxDensity());
        assertEquals(0.0, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(0.0, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor5() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(timestampSeconds);

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor6() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(time);

        // check default values
        assertNull(tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor7() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = new BodyKinematics();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, timestampSeconds);

        // check default values
        assertSame(kinematics, tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor8() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var kinematics = new BodyKinematics();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, time);

        // check default values
        assertSame(kinematics, tkb.getKinematics());
        assertNull(tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor9() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b, timestampSeconds);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(b, tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor10() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(b, time);

        // check default values
        assertNull(tkb.getKinematics());
        assertSame(b, tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertNull(tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor11() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = new BodyKinematics();
        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        // check default values
        assertSame(kinematics, tkb.getKinematics());
        assertSame(b, tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor12() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var kinematics = new BodyKinematics();
        final var b = new BodyMagneticFluxDensity();
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, time);

        // check default values
        assertSame(kinematics, tkb.getKinematics());
        assertSame(b, tkb.getMagneticFluxDensity());
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
        final var time1 = tkb.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time2);
        assertEquals(time1, time2);

        final var tk = tkb.getTimedKinematics();
        assertSame(kinematics, tk.getKinematics());
        assertEquals(timestampSeconds, tk.getTimestampSeconds(), 0.0);
    }

    @Test
    void testConstructor13() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();
        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(tkb1);

        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testGetSetKinematics() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(tkb.getKinematics());

        // set a new value
        final var kinematics = new BodyKinematics();
        tkb.setKinematics(kinematics);

        // check
        assertSame(kinematics, tkb.getKinematics());
    }

    @Test
    void testGetSetMagneticFluxDensity() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertNull(tkb.getMagneticFluxDensity());

        // set a new value
        final var b = new BodyMagneticFluxDensity();
        tkb.setMagneticFluxDensity(b);

        // check
        assertSame(b, tkb.getMagneticFluxDensity());
    }

    @Test
    void testGetSetTimestampSeconds() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        assertEquals(0.0, tkb.getTimestampSeconds(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        tkb.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(timestampSeconds, tkb.getTimestampSeconds(), 0.0);
    }

    @Test
    void testGetSetTimestamp() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        final var time1 = tkb.getTimestamp();

        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        tkb.setTimestamp(time2);

        final var time3 = tkb.getTimestamp();
        final var time4 = new Time(0.0, TimeUnit.HOUR);
        tkb.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    void testGetSetTimedKinematics() {
        final var tkb = new TimedBodyKinematicsAndMagneticFluxDensity();

        // check default value
        final var tk1 = tkb.getTimedKinematics();
        assertNull(tk1.getKinematics());
        assertEquals(0.0, tk1.getTimestampSeconds(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var tk2 = new TimedBodyKinematics(kinematics, timestampSeconds);
        tkb.setTimedKinematics(tk2);

        // check
        final var tk3 = tkb.getTimedKinematics();
        final var tk4 = new TimedBodyKinematics();
        tkb.getTimedKinematics(tk4);
        assertEquals(tk2, tk3);
        assertNotSame(tk2, tk3);
        assertEquals(tk2, tk4);
        assertNotSame(tk2, tk4);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();
        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity();

        tkb2.copyFrom(tkb1);

        // check
        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();
        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity();

        tkb1.copyTo(tkb2);

        // check
        assertEquals(tkb1, tkb2);
        assertEquals(tkb1.getKinematics(), tkb2.getKinematics());
        assertNotSame(tkb1.getKinematics(), tkb2.getKinematics());
        assertEquals(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertNotSame(tkb1.getMagneticFluxDensity(), tkb2.getMagneticFluxDensity());
        assertEquals(tkb1.getTimestampSeconds(), tkb2.getTimestampSeconds(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics2, b2, timestampSeconds2);

        assertEquals(tkb1.hashCode(), tkb2.hashCode());
        assertNotEquals(tkb1.hashCode(), tkb3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics2, b2, timestampSeconds2);

        //noinspection EqualsWithItself
        assertEquals(tkb1, tkb1);
        //noinspection EqualsWithItself
        assertTrue(tkb1.equals(tkb1));
        assertTrue(tkb1.equals(tkb2));
        assertFalse(tkb1.equals(tkb3));
        assertNotEquals(null, tkb1);
        assertFalse(tkb1.equals(null));
        assertNotEquals(new Object(), tkb1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics1 = createKinematics();
        final var b1 = createMagneticFlux();

        final var kinematics2 = createKinematics();
        final var b2 = createMagneticFlux();

        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb2 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics1, b1, timestampSeconds1);

        final var tkb3 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics2, b2, timestampSeconds2);

        assertTrue(tkb1.equals(tkb1, THRESHOLD));
        assertTrue(tkb1.equals(tkb2, THRESHOLD));
        assertFalse(tkb1.equals(tkb3, THRESHOLD));
        assertFalse(tkb1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();

        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        final var tkb2 = tkb1.clone();

        // check
        assertEquals(tkb1, tkb2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var kinematics = createKinematics();
        final var b = createMagneticFlux();

        final var tkb1 = new TimedBodyKinematicsAndMagneticFluxDensity(kinematics, b, timestampSeconds);

        final var bytes = SerializationHelper.serialize(tkb1);
        final var tkb2 = SerializationHelper.deserialize(bytes);

        assertEquals(tkb1, tkb2);
        assertNotSame(tkb1, tkb2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = TimedBodyKinematicsAndMagneticFluxDensity.class.getDeclaredField("serialVersionUID");
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
