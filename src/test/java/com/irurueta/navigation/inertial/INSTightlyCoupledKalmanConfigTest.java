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
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class INSTightlyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // check default values
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);

        // test constructor with values
        final Distance pseudoRangeSDDistance = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final Speed rangeRateSDSpeed = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        config = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSDDistance, rangeRateSDSpeed);

        // check default values
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);

        // test copy constructor
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig(config);

        // check default values
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testGetSetGyroNoisePSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerNoisePSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasPSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerBiasPSD(accelerometerBiasPSD);

        // check
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
    }

    @Test
    public void testGetSetGyroBiasPSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroBiasPSD(gyroBiasPSD);

        // check
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
    }

    @Test
    public void testGetSetClockFrequencyPSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
    }

    @Test
    public void testGetSetClockPhasePSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
    }

    @Test
    public void testGetSetRangeRateSD() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testSetValues() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testGetSetPseudoRangeSDDistance() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        final Distance distance1 = config.getPseudoRangeSDDistance();

        assertEquals(0.0, distance1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distance1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance distance2 = new Distance(pseudoRangeSD, DistanceUnit.METER);

        config.setPseudoRangeSD(distance2);

        // check
        final Distance distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getPseudoRangeSDDistance(distance3);
        final Distance distance4 = config.getPseudoRangeSDDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    public void testGetSetRangeRateSDSpeed() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default value
        final Speed speed1 = config.getRangeRateSDSpeed();

        assertEquals(0.0, speed1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speed1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Speed speed2 = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        config.setRangeRateSD(speed2);

        // check
        final Speed speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getRangeRateSDSpeed(speed3);
        final Speed speed4 = config.getRangeRateSDSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    public void testSetValues2() {
        final INSTightlyCoupledKalmanConfig config = new INSTightlyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance pseudoRangeSDDistance = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final Speed rangeRateSDSpeed = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSDDistance, rangeRateSDSpeed);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(clockFrequencyPSD, config2.getClockFrequencyPSD(), 0.0);
        assertEquals(clockPhasePSD, config2.getClockPhasePSD(), 0.0);
        assertEquals(pseudoRangeSD, config2.getPseudoRangeSD(), 0.0);
        assertEquals(rangeRateSD, config2.getRangeRateSD(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config3 = new INSTightlyCoupledKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    public void testEquals() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config3 = new INSTightlyCoupledKalmanConfig();

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
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);
        final INSTightlyCoupledKalmanConfig config3 = new INSTightlyCoupledKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws CloneNotSupportedException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);

        final Object config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSTightlyCoupledKalmanConfig config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, clockFrequencyPSD, clockPhasePSD,
                pseudoRangeSD, rangeRateSD);

        final byte[] bytes = SerializationHelper.serialize(config1);

        final INSTightlyCoupledKalmanConfig config2 = SerializationHelper.deserialize(bytes);

        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = INSTightlyCoupledKalmanConfig.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
