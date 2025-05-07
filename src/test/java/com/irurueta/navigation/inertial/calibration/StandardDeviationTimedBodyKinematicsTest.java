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
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class StandardDeviationTimedBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_TIMESTAMP_VALUE = -10.0;
    private static final double MAX_TIMESTAMP_VALUE = 10.0;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor1() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor2() {
        final var kinematics = createBodyKinematics();
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics);

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor3() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds);

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor4() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time);

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor5() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds);

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor6() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time);

        // check default values
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testConstructor7() {
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(),
                specificForceStandardDeviation, 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(
                specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor8() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics,
                specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor9() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(timestampSeconds,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(timestampSeconds,
                specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor10() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(time,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(time,
                specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor11() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(
                kinematics, timestampSeconds, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(
                kinematics, timestampSeconds, specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor12() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, time,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, time,
                specificForceStandardDeviation, -1.0));
    }

    @Test
    void testConstructor13() {
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(a, w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(a, w2));
    }

    @Test
    void testConstructor14() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, a, w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, a, w2));
    }

    @Test
    void testConstructor15() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(timestampSeconds, a,
                w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(timestampSeconds,
                a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(timestampSeconds,
                a, w2));
    }

    @Test
    void testConstructor16() {
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(time, a, w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals( angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(time, a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(time, a, w2));
    }

    @Test
    void testConstructor17() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, a, w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(timestampSeconds, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, a, w2));
    }

    @Test
    void testConstructor18() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics(kinematics, time, a,
                w);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time2);
        assertEquals(time1, time2);

        // Force IllegalArgumentException
        final var a2 = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, time,
                a2, w));
        final var w2 = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationTimedBodyKinematics(kinematics, time,
                a, w2));
    }

    @Test
    void testCopyConstructor() {
        final var kinematics = createBodyKinematics();
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var time = new Time(timestampSeconds, TimeUnit.SECOND);
        final var a = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var w = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics, time, a,
                w);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(
                standardDeviationTimedBodyKinematics1);

        // check default values
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        final var acceleration1 = standardDeviationTimedBodyKinematics2
                .getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(specificForceStandardDeviation, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
        final var angularSpeed1 = standardDeviationTimedBodyKinematics2
                .getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(angularRateStandardDeviation, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());
        final var angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(kinematics, standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        final var time1 = standardDeviationTimedBodyKinematics2.getTimestamp();
        assertEquals(time, time1);
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics2.getTimestamp(time2);
        assertEquals(time1, time2);
    }

    @Test
    void testGetSetSpecificForceStandardDeviation() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(specificForceStandardDeviation);

        // check
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(-1.0));
    }

    @Test
    void testGetSetSpecificForceStandardDeviationAcceleration() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        final var acceleration1 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();

        assertEquals(0.0, acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var acceleration2 = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(acceleration2);

        // check
        final var acceleration3 = standardDeviationTimedBodyKinematics
                .getSpecificForceStandardDeviationAsAcceleration();
        final var acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        standardDeviationTimedBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);

        // Force IllegalArgumentException
        final var a = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertThrows(IllegalArgumentException.class,
                () -> standardDeviationTimedBodyKinematics.setSpecificForceStandardDeviation(a));
    }

    @Test
    void testGetSetAngularRateStandardDeviation() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(angularRateStandardDeviation);

        // check
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics.getAngularRateStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(-1.0));
    }

    @Test
    void testGetSetAngularRateStandardDeviationAsAngularSpeed() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        final var angularSpeed1 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();

        assertEquals(0.0, angularSpeed1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, angularSpeed1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularSpeed2 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        standardDeviationTimedBodyKinematics.setAngularRateStandardDeviation(angularSpeed2);

        // check
        final var angularSpeed3 = standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        final var angularSpeed4 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        standardDeviationTimedBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    void testGetSetKinematics() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        assertNull(standardDeviationTimedBodyKinematics.getKinematics());

        // set a new value
        final var kinematics = new BodyKinematics();
        standardDeviationTimedBodyKinematics.setKinematics(kinematics);

        // check
        assertSame(kinematics, standardDeviationTimedBodyKinematics.getKinematics());
    }

    @Test
    void testGetSetTimestampSeconds() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        assertEquals(0.0, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        standardDeviationTimedBodyKinematics.setTimestampSeconds(timestampSeconds);

        // check
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics.getTimestampSeconds(), 0.0);
    }

    @Test
    void testGetSetTimestamp() {
        final var standardDeviationTimedBodyKinematics = new StandardDeviationTimedBodyKinematics();

        // check default value
        final var time1 = standardDeviationTimedBodyKinematics.getTimestamp();

        assertEquals(0.0, time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var time2 = new Time(timestampSeconds, TimeUnit.SECOND);
        standardDeviationTimedBodyKinematics.setTimestamp(time2);

        final var time3 = standardDeviationTimedBodyKinematics.getTimestamp();
        final var time4 = new Time(0.0, TimeUnit.HOUR);
        standardDeviationTimedBodyKinematics.getTimestamp(time4);

        // check
        assertEquals(time2, time3);
        assertEquals(time2, time4);
    }

    @Test
    void testCopyFromWhenBodyKinematicsIsAvailableAtSourceAndDestinationIsEmpty() {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics();

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertEquals(kinematics, standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        assertEquals(specificForceStandardDeviation,
                standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(angularRateStandardDeviation,
                standardDeviationTimedBodyKinematics1.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenDestinationHasKinematics() {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics();
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertNull(standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(0.0, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        assertEquals(0.0, standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(0.0, standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenSourceAndDestinationHasKinematics() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        standardDeviationTimedBodyKinematics2.copyFrom(standardDeviationTimedBodyKinematics1);

        // check
        assertEquals(kinematics1, standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        assertEquals(specificForceStandardDeviation1,
                standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(angularRateStandardDeviation1,
                standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        standardDeviationTimedBodyKinematics1.copyTo(standardDeviationTimedBodyKinematics2);

        // check
        assertEquals(kinematics1, standardDeviationTimedBodyKinematics2.getKinematics());
        assertEquals(timestampSeconds1, standardDeviationTimedBodyKinematics2.getTimestampSeconds(), 0.0);
        assertEquals(specificForceStandardDeviation1,
                standardDeviationTimedBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(angularRateStandardDeviation1,
                standardDeviationTimedBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    void testHashCode() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics3 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        assertEquals(standardDeviationTimedBodyKinematics1.hashCode(),
                standardDeviationTimedBodyKinematics2.hashCode());
        assertNotEquals(standardDeviationTimedBodyKinematics1.hashCode(),
                standardDeviationTimedBodyKinematics3.hashCode());
    }

    @Test
    void testEquals() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics3 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        //noinspection EqualsWithItself
        assertEquals(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics1);
        //noinspection EqualsWithItself
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics1));
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics2));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics3));
        assertNotEquals(null, standardDeviationTimedBodyKinematics1);
        assertNotEquals(new Object(), standardDeviationTimedBodyKinematics1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var kinematics1 = createBodyKinematics();
        final var kinematics2 = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds1 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var timestampSeconds2 = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);

        final var specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        final var angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics2 = new StandardDeviationTimedBodyKinematics(kinematics1,
                timestampSeconds1, specificForceStandardDeviation1, angularRateStandardDeviation1);
        final var standardDeviationTimedBodyKinematics3 = new StandardDeviationTimedBodyKinematics(kinematics2,
                timestampSeconds2, specificForceStandardDeviation2, angularRateStandardDeviation2);

        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics1, THRESHOLD));
        assertTrue(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics2, THRESHOLD));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(standardDeviationTimedBodyKinematics3, THRESHOLD));
        assertFalse(standardDeviationTimedBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        final var standardDeviationTimedBodyKinematics2 = standardDeviationTimedBodyKinematics1.clone();

        // check
        assertEquals(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var kinematics = createBodyKinematics();

        final var randomizer = new UniformRandomizer();
        final var timestampSeconds = randomizer.nextDouble(MIN_TIMESTAMP_VALUE, MAX_TIMESTAMP_VALUE);
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final var standardDeviationTimedBodyKinematics1 = new StandardDeviationTimedBodyKinematics(kinematics,
                timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        final var bytes = SerializationHelper.serialize(standardDeviationTimedBodyKinematics1);

        final var standardDeviationTimedBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
        assertNotSame(standardDeviationTimedBodyKinematics1, standardDeviationTimedBodyKinematics2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = StandardDeviationTimedBodyKinematics.class.getDeclaredField("serialVersionUID");
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
