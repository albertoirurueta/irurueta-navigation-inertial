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
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class INSLooselyCoupledKalmanInitializerConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default values
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config = new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);

        // check default values
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // test constructor with measurement values
        final Angle initialAttitudeUncertaintyAngle = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final Speed initialVelocityUncertaintySpeed = new Speed(initialVelocityUncertainty,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialPositionUncertaintyDistance = new Distance(initialPositionUncertainty,
                DistanceUnit.METER);
        final Acceleration initialAccelerationBiasUncertaintyAcceleration = new Acceleration(
                initialAccelerationBiasUncertainty, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed initialGyroscopeBiasUncertaintyAngularSpeed = new AngularSpeed(
                initialGyroscopeBiasUncertainty, AngularSpeedUnit.RADIANS_PER_SECOND);
        config = new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertaintyAngle,
                initialVelocityUncertaintySpeed, initialPositionUncertaintyDistance,
                initialAccelerationBiasUncertaintyAcceleration, initialGyroscopeBiasUncertaintyAngularSpeed);

        // check default values
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // test copy constructor
        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig(config);

        // check default values
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialAttitudeUncertainty() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialAttitudeUncertaintyAngle() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        final Angle initialAttitudeUncertainty1 = config.getInitialAttitudeUncertaintyAngle();

        assertEquals(0.0, initialAttitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, initialAttitudeUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Angle initialAttitudeUncertainty2 = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        config.setInitialAttitudeUncertainty(initialAttitudeUncertainty2);

        // check
        final Angle initialAttitudeUncertainty3 = config.getInitialAttitudeUncertaintyAngle();
        final Angle initialAttitudeUncertainty4 = new Angle(0.0, AngleUnit.DEGREES);
        config.getInitialAttitudeUncertaintyAngle(initialAttitudeUncertainty4);

        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty3);
        assertEquals(initialAttitudeUncertainty2, initialAttitudeUncertainty4);
    }

    @Test
    public void testGetSetInitialVelocityUncertainty() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty);

        // check
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialVelocityUncertaintySpeed() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        final Speed initialVelocityUncertaintySpeed1 = config.getInitialVelocityUncertaintySpeed();

        assertEquals(0.0, initialVelocityUncertaintySpeed1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, initialVelocityUncertaintySpeed1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Speed initialVelocityUncertainty2 = new Speed(initialVelocityUncertainty, SpeedUnit.METERS_PER_SECOND);
        config.setInitialVelocityUncertainty(initialVelocityUncertainty2);

        // check
        final Speed initialVelocityUncertainty3 = config.getInitialVelocityUncertaintySpeed();
        final Speed initialVelocityUncertainty4 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getInitialVelocityUncertaintySpeed(initialVelocityUncertainty4);

        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty3);
        assertEquals(initialVelocityUncertainty2, initialVelocityUncertainty4);
    }

    @Test
    public void testGetSetInitialPositionUncertainty() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialPositionUncertainty(initialPositionUncertainty);

        // check
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialPositionUncertaintyDistance() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        final Distance initialPositionUncertainty1 = config.getInitialPositionUncertaintyDistance();

        assertEquals(0.0, initialPositionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, initialPositionUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance initialPositionUncertainty2 = new Distance(initialPositionUncertainty, DistanceUnit.METER);
        config.setInitialPositionUncertainty(initialPositionUncertainty2);

        // check
        final Distance initialPositionUncertainty3 = config.getInitialPositionUncertaintyDistance();
        final Distance initialPositionUncertainty4 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getInitialPositionUncertaintyDistance(initialPositionUncertainty4);

        assertEquals(initialPositionUncertainty2, initialPositionUncertainty3);
        assertEquals(initialPositionUncertainty2, initialPositionUncertainty4);
    }

    @Test
    public void testGetSetInitialAccelerationBiasUncertainty() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialAccelerationBiasUncertainty(initialAccelerationBiasUncertainty);

        // check
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialAccelerationBiasUncertaintyAcceleration() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        final Acceleration initialAccelerationBiasUncertainty1 =
                config.getInitialAccelerationBiasUncertaintyAcceleration();

        assertEquals(0.0, initialAccelerationBiasUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, initialAccelerationBiasUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Acceleration initialAccelerationBiasUncertainty2 = new Acceleration(initialAccelerationBiasUncertainty,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        config.setInitialAccelerationBiasUncertainty(initialAccelerationBiasUncertainty2);

        // check
        final Acceleration initialAccelerationBiasUncertainty3 =
                config.getInitialAccelerationBiasUncertaintyAcceleration();
        final Acceleration initialAccelerationBiasUncertainty4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        config.getInitialAccelerationBiasUncertaintyAcceleration(initialAccelerationBiasUncertainty4);

        assertEquals(initialAccelerationBiasUncertainty2, initialAccelerationBiasUncertainty3);
        assertEquals(initialAccelerationBiasUncertainty2, initialAccelerationBiasUncertainty4);
    }

    @Test
    public void testGetSetInitialGyroscopeBiasUncertainty() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setInitialGyroscopeBiasUncertainty(initialGyroscopeBiasUncertainty);

        // check
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testGetSetInitialGyroscopeBiasUncertaintyAngularSpeed() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default value
        final AngularSpeed initialGyroscopeBiasUncertainty1 = config.getInitialGyroscopeBiasUncertaintyAngularSpeed();

        assertEquals(0.0, initialGyroscopeBiasUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, initialGyroscopeBiasUncertainty1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final AngularSpeed initialGyroscopeBiasUncertainty2 = new AngularSpeed(initialGyroscopeBiasUncertainty,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        config.setInitialGyroscopeBiasUncertainty(initialGyroscopeBiasUncertainty2);

        // check
        final AngularSpeed initialGyroscopeBiasUncertainty3 = config.getInitialGyroscopeBiasUncertaintyAngularSpeed();
        final AngularSpeed initialGyroscopeBiasUncertainty4 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        config.getInitialGyroscopeBiasUncertaintyAngularSpeed(initialGyroscopeBiasUncertainty4);

        assertEquals(initialGyroscopeBiasUncertainty2, initialGyroscopeBiasUncertainty3);
        assertEquals(initialGyroscopeBiasUncertainty2, initialGyroscopeBiasUncertainty4);
    }

    @Test
    public void testSetValues() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default values
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setValues(initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testSetValues2() {
        final INSLooselyCoupledKalmanInitializerConfig config = new INSLooselyCoupledKalmanInitializerConfig();

        // check default values
        assertEquals(0.0, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(0.0, config.getInitialGyroscopeBiasUncertainty(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Angle initialAttitudeUncertaintyAngle = new Angle(initialAttitudeUncertainty, AngleUnit.RADIANS);
        final Speed initialVelocityUncertaintySpeed = new Speed(initialVelocityUncertainty,
                SpeedUnit.METERS_PER_SECOND);
        final Distance initialPositionUncertaintyDistance = new Distance(initialPositionUncertainty,
                DistanceUnit.METER);
        final Acceleration initialAccelerationBiasUncertaintyAcceleration =
                new Acceleration(initialAccelerationBiasUncertainty, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed initialGyroscopeBiasUncertaintyAngularSpeed = new AngularSpeed(
                initialGyroscopeBiasUncertainty, AngularSpeedUnit.RADIANS_PER_SECOND);
        config.setValues(initialAttitudeUncertaintyAngle, initialVelocityUncertaintySpeed,
                initialPositionUncertaintyDistance, initialAccelerationBiasUncertaintyAcceleration,
                initialGyroscopeBiasUncertaintyAngularSpeed);

        // check
        assertEquals(initialAttitudeUncertainty, config.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);

        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig();

        config1.copyTo(config2);

        // check
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);

        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(initialAttitudeUncertainty, config2.getInitialAttitudeUncertainty(), 0.0);
        assertEquals(initialVelocityUncertainty, config2.getInitialVelocityUncertainty(), 0.0);
        assertEquals(initialPositionUncertainty, config2.getInitialPositionUncertainty(), 0.0);
        assertEquals(initialAccelerationBiasUncertainty, config2.getInitialAccelerationBiasUncertainty(), 0.0);
        assertEquals(initialGyroscopeBiasUncertainty, config2.getInitialGyroscopeBiasUncertainty(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config3 = new INSLooselyCoupledKalmanInitializerConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config3 = new INSLooselyCoupledKalmanInitializerConfig();

        //noinspection EqualsWithItself
        assertEquals(config1, config1);
        //noinspection EqualsWithItself
        assertTrue(config1.equals(config1));
        assertTrue(config1.equals(config2));
        assertFalse(config1.equals(config3));
        assertNotEquals(config1, null);
        assertFalse(config1.equals(null));
        assertNotEquals(config1, new Object());
    }

    @Test
    public void testEqualsWithThreshold() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config2 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final INSLooselyCoupledKalmanInitializerConfig config3 = new INSLooselyCoupledKalmanInitializerConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final INSLooselyCoupledKalmanInitializerConfig config1 = new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty, initialPositionUncertainty,
                initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);

        final byte[] bytes = SerializationHelper.serialize(config1);
        final INSLooselyCoupledKalmanInitializerConfig config2 = SerializationHelper.deserialize(bytes);

        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = INSLooselyCoupledKalmanInitializerConfig.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
