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

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class INSTightlyCoupledKalmanInitializerConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {

        // test empty constructor
        var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default values
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // test constructor with values
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty);

        // check default values
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);

        // test constructor with measurement values
        final var initialAttitudeUncertaintyAngle = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final var initialVelocityUncertaintySpeed = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final var initialPositionUncertaintyDistance = new Distance(initialPositionUncertainty, 
                DistanceUnit.METER);
        final var initialAccelerationBiasUncertaintyAcceleration = new Acceleration(initialAccelerationBiasUncertainty,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialGyroscopeBiasUncertaintyAngularSpeed = new AngularSpeed(initialGyroscopeBiasUncertainty,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final var initialClockOffsetUncertaintyDistance = new Distance(initialClockOffsetUncertainty, 
                DistanceUnit.METER);
        final var initialClockDriftUncertaintySpeed = new Speed(initialClockDriftUncertainty, 
                SpeedUnit.METERS_PER_SECOND);
        config = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertaintyAngle,
                initialVelocityUncertaintySpeed, initialPositionUncertaintyDistance,
                initialAccelerationBiasUncertaintyAcceleration, initialGyroscopeBiasUncertaintyAngularSpeed,
                initialClockOffsetUncertaintyDistance, initialClockDriftUncertaintySpeed);

        // check default values
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);

        // test copy constructor
        final var config2 = new INSTightlyCoupledKalmanInitializerConfig(config);

        // check default values
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialAttitudeUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialAttitudeUncertaintyAngle() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialAttitudeUncertainty1 = config.getInitialAttitudeUncertaintyAngle();

        assertEquals(0.0, initialAttitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, initialAttitudeUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAttitudeUncertainty2 = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty2);

        // check
        final var initialAttitudeUncertainty3 = config.getInitialAttitudeUncertaintyAngle();
        final var initialAttitudeUncertainty4 = new Angle(0.0, AngleUnit.DEGREES);
        config.getInitialAttitudeUncertaintyAngle(initialAttitudeUncertainty4);

        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty3);
        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty4);
    }

    @Test
    void testGetSetInitialVelocityUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty);

        // check
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialVelocityUncertaintySpeed() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialVelocityUncertaintySpeed1 = config.getInitialVelocityUncertaintySpeed();

        assertEquals(0.0, initialVelocityUncertaintySpeed1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialVelocityUncertaintySpeed1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty2 = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty2);

        // check
        final var initialVelocityUncertainty3 = config.getInitialVelocityUncertaintySpeed();
        final var initialVelocityUncertainty4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialVelocityUncertaintySpeed(initialVelocityUncertainty4);

        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty3);
        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty4);
    }

    @Test
    void testGetSetInitialPositionUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialPositionUncertainty(initialPositionUncertainty);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialPositionUncertaintyDistance() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialPositionUncertainty1 = config.getInitialPositionUncertaintyDistance();

        assertEquals(0.0, initialPositionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialPositionUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty2 = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        config.setInitialPositionUncertainty(initialPositionUncertainty2);

        // check
        final var initialPositionUncertainty3 = config.getInitialPositionUncertaintyDistance();
        final var initialPositionUncertainty4 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialPositionUncertaintyDistance(initialPositionUncertainty4);

        assertEquals(initialPositionUncertainty2, initialPositionUncertainty3);
        assertEquals(initialPositionUncertainty2, initialPositionUncertainty4);
    }

    @Test
    void testGetSetInitialAccelerationBiasUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialAccelerationBiasUncertainty(initialAccelerationBiasUncertainty);

        // check
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialAccelerationBiasUncertaintyAcceleration() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialAccelerationBiasUncertainty1 = config.getInitialAccelerationBiasUncertaintyAcceleration();

        assertEquals(0.0, initialAccelerationBiasUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialAccelerationBiasUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty2 = new Acceleration(initialAccelerationBiasUncertainty,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        config.setInitialAccelerationBiasUncertainty(initialAccelerationBiasUncertainty2);

        // check
        final var initialAccelerationBiasUncertainty3 = config.getInitialAccelerationBiasUncertaintyAcceleration();
        final var initialAccelerationBiasUncertainty4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        config.getInitialAccelerationBiasUncertaintyAcceleration(initialAccelerationBiasUncertainty4);

        assertEquals(initialAccelerationBiasUncertainty2, initialAccelerationBiasUncertainty3);
        assertEquals(initialAccelerationBiasUncertainty2, initialAccelerationBiasUncertainty4);
    }

    @Test
    void testGetSetInitialGyroscopeBiasUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialGyroscopeBiasUncertainty(initialGyroscopeBiasUncertainty);

        // check
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialGyroscopeBiasUncertaintyAngularSpeed() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialGyroscopeBiasUncertainty1 = config.getInitialGyroscopeBiasUncertaintyAngularSpeed();

        assertEquals(0.0, initialGyroscopeBiasUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialGyroscopeBiasUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty2 = new AngularSpeed(initialGyroscopeBiasUncertainty,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        config.setInitialGyroscopeBiasUncertainty(initialGyroscopeBiasUncertainty2);

        // check
        final var initialGyroscopeBiasUncertainty3 = config.getInitialGyroscopeBiasUncertaintyAngularSpeed();
        final var initialGyroscopeBiasUncertainty4 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        config.getInitialGyroscopeBiasUncertaintyAngularSpeed(initialGyroscopeBiasUncertainty4);

        assertEquals(initialGyroscopeBiasUncertainty2, initialGyroscopeBiasUncertainty3);
        assertEquals(initialGyroscopeBiasUncertainty2, initialGyroscopeBiasUncertainty4);
    }

    @Test
    void testGetSetInitialClockOffsetUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty);

        // check
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialClockOffsetUncertaintyDistance() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialClockOffsetUncertainty1 = config.getInitialClockOffsetUncertaintyDistance();

        assertEquals(0.0, initialClockOffsetUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialClockOffsetUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty2 = new Distance(initialClockOffsetUncertainty, DistanceUnit.METER);
        config.setInitialClockOffsetUncertainty(initialClockOffsetUncertainty2);

        // check
        final var initialClockOffsetUncertainty3 = config.getInitialClockOffsetUncertaintyDistance();
        final var initialClockOffsetUncertainty4 = new Distance(0.0, DistanceUnit.METER);
        config.getInitialClockOffsetUncertaintyDistance(initialClockOffsetUncertainty4);

        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty3);
        assertEquals(initialClockOffsetUncertainty2, initialClockOffsetUncertainty4);
    }

    @Test
    void testGetSetInitialClockDriftUncertainty() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty);

        // check
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testGetSetInitialClockDriftUncertaintySpeed() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        final var initialClockDriftUncertainty1 = config.getInitialClockDriftUncertaintySpeed();

        assertEquals(0.0, initialClockDriftUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialClockDriftUncertainty1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty2 = new Speed(initialClockDriftUncertainty,
                SpeedUnit.METERS_PER_SECOND);
        config.setInitialClockDriftUncertainty(initialClockDriftUncertainty2);

        // check
        final var initialClockDriftUncertainty3 = config.getInitialClockDriftUncertaintySpeed();
        final var initialClockDriftUncertainty4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialClockDriftUncertaintySpeed(initialClockDriftUncertainty4);

        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty3);
        assertEquals(initialClockDriftUncertainty2, initialClockDriftUncertainty4);
    }

    @Test
    void testSetValues() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setValues(initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testSetValues2() {
        final var config = new INSTightlyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialClockDriftUncertainty(), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var initialAttitudeUncertaintyAngle = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final var initialVelocityUncertaintySpeed = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        final var initialPositionUncertaintyDistance = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        final var initialAccelerationBiasUncertaintyAcceleration = new Acceleration(initialAccelerationBiasUncertainty,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var initialGyroscopeBiasUncertaintyAngularSpeed = new AngularSpeed(initialGyroscopeBiasUncertainty,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final var initialClockOffsetUncertaintyDistance = new Distance(initialClockOffsetUncertainty,
                DistanceUnit.METER);
        final var initialClockDriftUncertaintySpeed = new Speed(initialClockDriftUncertainty,
                SpeedUnit.METERS_PER_SECOND);

        config.setValues(initialAttitudeUncertaintyAngle, initialVelocityUncertaintySpeed,
                initialPositionUncertaintyDistance, initialAccelerationBiasUncertaintyAcceleration,
                initialGyroscopeBiasUncertaintyAngularSpeed, initialClockOffsetUncertaintyDistance,
                initialClockDriftUncertaintySpeed);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);

        final var config2 = new INSTightlyCoupledKalmanInitializerConfig();

        config1.copyTo(config2);

        // check
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);

        final var config2 = new INSTightlyCoupledKalmanInitializerConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
        assertEquals(initialClockOffsetUncertainty, config2.getInitialClockOffsetUncertainty(), 0.0);
        assertEquals(initialClockDriftUncertainty, config2.getInitialClockDriftUncertainty(), 0.0);
    }

    @Test
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config2 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config3 = new INSTightlyCoupledKalmanInitializerConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config2 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config3 = new INSTightlyCoupledKalmanInitializerConfig();

        //noinspection EqualsWithItself
        assertEquals(config1, config1);
        //noinspection EqualsWithItself
        assertTrue(config1.equals(config1));
        assertTrue(config1.equals(config2));
        assertFalse(config1.equals(config3));
        assertNotEquals(null, config1);
        assertFalse(config1.equals(null));
        assertNotEquals(new Object(), config1);
    }

    @Test
    void testEqualsWithThreshold() {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config2 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config3 = new INSTightlyCoupledKalmanInitializerConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);
        final var config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var config1 = new INSTightlyCoupledKalmanInitializerConfig(initialAttitudeUncertainty,
                initialVelocityUncertainty, initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty, initialClockOffsetUncertainty, initialClockDriftUncertainty);

        final var bytes = SerializationHelper.serialize(config1);

        final var config2 = SerializationHelper.deserialize(bytes);

        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = INSTightlyCoupledKalmanInitializerConfig.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
