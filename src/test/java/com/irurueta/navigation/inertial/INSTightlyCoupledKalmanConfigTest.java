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
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class INSTightlyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    void testConstructor() {
        // test empty constructor
        var config = new INSTightlyCoupledKalmanConfig();

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
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

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
        final var pseudoRangeSDDistance = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final var rangeRateSDSpeed = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

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
        final var config2 = new INSTightlyCoupledKalmanConfig(config);

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
    void testGetSetGyroNoisePSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroNoisePSD(gyroNoisePSD);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
    }

    @Test
    void testGetSetAccelerometerNoisePSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerNoisePSD(accelerometerNoisePSD);

        // check
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
    }

    @Test
    void testGetSetAccelerometerBiasPSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setAccelerometerBiasPSD(accelerometerBiasPSD);

        // check
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
    }

    @Test
    void testGetSetGyroBiasPSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setGyroBiasPSD(gyroBiasPSD);

        // check
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
    }

    @Test
    void testGetSetClockFrequencyPSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockFrequencyPSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockFrequencyPSD(clockFrequencyPSD);

        // check
        assertEquals(clockFrequencyPSD, config.getClockFrequencyPSD(), 0.0);
    }

    @Test
    void testGetSetClockPhasePSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getClockPhasePSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setClockPhasePSD(clockPhasePSD);

        // check
        assertEquals(clockPhasePSD, config.getClockPhasePSD(), 0.0);
    }

    @Test
    void testGetSetPseudoRangeSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getPseudoRangeSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setPseudoRangeSD(pseudoRangeSD);

        // check
        assertEquals(pseudoRangeSD, config.getPseudoRangeSD(), 0.0);
    }

    @Test
    void testGetSetRangeRateSD() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getRangeRateSD(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setRangeRateSD(rangeRateSD);

        // check
        assertEquals(rangeRateSD, config.getRangeRateSD(), 0.0);
    }

    @Test
    void testSetValues() {
        final var config = new INSTightlyCoupledKalmanConfig();

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
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

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
    void testGetSetPseudoRangeSDDistance() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        final var distance1 = config.getPseudoRangeSDDistance();

        assertEquals(0.0, distance1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, distance1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var distance2 = new Distance(pseudoRangeSD, DistanceUnit.METER);

        config.setPseudoRangeSD(distance2);

        // check
        final var distance3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getPseudoRangeSDDistance(distance3);
        final var distance4 = config.getPseudoRangeSDDistance();

        assertEquals(distance2, distance3);
        assertEquals(distance2, distance4);
    }

    @Test
    void testGetSetRangeRateSDSpeed() {
        final var config = new INSTightlyCoupledKalmanConfig();

        // check default value
        final var speed1 = config.getRangeRateSDSpeed();

        assertEquals(0.0, speed1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speed1.getUnit());

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var speed2 = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

        config.setRangeRateSD(speed2);

        // check
        final var speed3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getRangeRateSDSpeed(speed3);
        final var speed4 = config.getRangeRateSDSpeed();

        assertEquals(speed2, speed3);
        assertEquals(speed2, speed4);
    }

    @Test
    void testSetValues2() {
        final var config = new INSTightlyCoupledKalmanConfig();

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
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var pseudoRangeSDDistance = new Distance(pseudoRangeSD, DistanceUnit.METER);
        final var rangeRateSDSpeed = new Speed(rangeRateSD, SpeedUnit.METERS_PER_SECOND);

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
    void testCopyTo() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new INSTightlyCoupledKalmanConfig();

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
    void testCopyFrom() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new INSTightlyCoupledKalmanConfig();

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
    void testHashCode() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new INSTightlyCoupledKalmanConfig();

        assertEquals(config1.hashCode(), config2.hashCode());
        assertNotEquals(config1.hashCode(), config3.hashCode());
    }

    @Test
    void testEquals() {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new INSTightlyCoupledKalmanConfig();

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
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config2 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);
        final var config3 = new INSTightlyCoupledKalmanConfig();

        assertTrue(config1.equals(config1, THRESHOLD));
        assertTrue(config1.equals(config2, THRESHOLD));
        assertFalse(config1.equals(config3, THRESHOLD));
        assertFalse(config1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws CloneNotSupportedException {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final var config2 = config1.clone();

        assertEquals(config1, config2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final var config1 = new INSTightlyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, clockFrequencyPSD, clockPhasePSD, pseudoRangeSD, rangeRateSD);

        final var bytes = SerializationHelper.serialize(config1);

        final var config2 = SerializationHelper.deserialize(bytes);

        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = INSTightlyCoupledKalmanConfig.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
