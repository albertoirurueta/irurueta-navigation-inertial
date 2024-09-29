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

public class INSLooselyCoupledKalmanConfigTest {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getPositionNoiseSD(), 0.0);
        assertEquals(0.0, config.getVelocityNoiseSD(), 0.0);

        // test constructor with values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config = new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

        // check default values
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config.getVelocityNoiseSD(), 0.0);

        // test constructor with values
        final Distance positionNoiseSDDistance = new Distance(positionNoiseSD, DistanceUnit.METER);
        final Speed velocityNoiseSDSpeed = new Speed(velocityNoiseSD, SpeedUnit.METERS_PER_SECOND);

        config = new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD,
                gyroBiasPSD, positionNoiseSDDistance, velocityNoiseSDSpeed);

        // check default values
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config.getVelocityNoiseSD(), 0.0);

        // test copy constructor
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig(config);

        // check default values
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config2.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config2.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testGetSetGyroNoisePSD() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

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
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

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
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

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
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

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
    public void testGetSetPositionNoiseSD() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getPositionNoiseSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setPositionNoiseSD(positionNoiseSD);

        // check
        assertEquals(positionNoiseSD, config.getPositionNoiseSD(), 0.0);
    }

    @Test
    public void testGetSetVelocityNoiseSD() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default value
        assertEquals(0.0, config.getVelocityNoiseSD(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        config.setVelocityNoiseSD(velocityNoiseSD);

        // check
        assertEquals(velocityNoiseSD, config.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testSetValues() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getPositionNoiseSD(), 0.0);
        assertEquals(0.0, config.getVelocityNoiseSD(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD,
                velocityNoiseSD);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testGetSetPositionNoiseSDAsDistance() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default value
        final Distance positionNoise1 = config.getPositionNoiseSDAsDistance();

        assertEquals(0.0, positionNoise1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoise1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Distance positionNoise2 = new Distance(positionNoiseSD, DistanceUnit.METER);

        config.setPositionNoiseSD(positionNoise2);

        // check
        final Distance positionNoise3 = new Distance(0.0, DistanceUnit.KILOMETER);
        config.getPositionNoiseSDAsDistance(positionNoise3);
        final Distance positionNoise4 = config.getPositionNoiseSDAsDistance();

        assertEquals(positionNoise2, positionNoise3);
        assertEquals(positionNoise2, positionNoise4);
    }

    @Test
    public void testGetSetVelocityNoiseSDAsSpeed() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default value
        final Speed velocityNoise1 = config.getVelocityNoiseSDAsSpeed();

        assertEquals(0.0, velocityNoise1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoise1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final Speed velocityNoise2 = new Speed(velocityNoiseSD, SpeedUnit.METERS_PER_SECOND);

        config.setVelocityNoiseSD(velocityNoise2);

        // check
        final Speed velocityNoise3 = new Speed(0.0, SpeedUnit.KILOMETERS_PER_HOUR);
        config.getVelocityNoiseSDAsSpeed(velocityNoise3);
        final Speed velocityNoise4 = config.getVelocityNoiseSDAsSpeed();

        assertEquals(velocityNoise2, velocityNoise3);
        assertEquals(velocityNoise2, velocityNoise4);
    }

    @Test
    public void testSetValues2() {
        final INSLooselyCoupledKalmanConfig config = new INSLooselyCoupledKalmanConfig();

        // check default values
        assertEquals(0.0, config.getGyroNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(0.0, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, config.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, config.getPositionNoiseSD(), 0.0);
        assertEquals(0.0, config.getVelocityNoiseSD(), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final Distance positionNoiseSDDistance = new Distance(positionNoiseSD, DistanceUnit.METER);
        final Speed velocityNoiseSDSpeed = new Speed(velocityNoiseSD, SpeedUnit.METERS_PER_SECOND);

        config.setValues(gyroNoisePSD, accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD,
                positionNoiseSDDistance, velocityNoiseSDSpeed);

        // check
        assertEquals(gyroNoisePSD, config.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testCopyTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig();

        config1.copyTo(config2);

        // check
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config2.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config2.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testCopyFrom() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig();

        config2.copyFrom(config1);

        // check
        assertEquals(gyroNoisePSD, config2.getGyroNoisePSD(), 0.0);
        assertEquals(accelerometerNoisePSD, config2.getAccelerometerNoisePSD(), 0.0);
        assertEquals(accelerometerBiasPSD, config2.getAccelerometerBiasPSD(), 0.0);
        assertEquals(gyroBiasPSD, config2.getGyroBiasPSD(), 0.0);
        assertEquals(positionNoiseSD, config2.getPositionNoiseSD(), 0.0);
        assertEquals(velocityNoiseSD, config2.getVelocityNoiseSD(), 0.0);
    }

    @Test
    public void testHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 = new INSLooselyCoupledKalmanConfig();

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
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 = new INSLooselyCoupledKalmanConfig();

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
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config2 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
        final INSLooselyCoupledKalmanConfig config3 = new INSLooselyCoupledKalmanConfig();

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
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

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
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        final INSLooselyCoupledKalmanConfig config1 = new INSLooselyCoupledKalmanConfig(gyroNoisePSD,
                accelerometerNoisePSD, accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);

        final byte[] bytes = SerializationHelper.serialize(config1);

        final INSLooselyCoupledKalmanConfig config2 = SerializationHelper.deserialize(bytes);

        assertEquals(config1, config2);
        assertNotSame(config1, config2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = INSLooselyCoupledKalmanConfig.class.getDeclaredField("serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }
}
