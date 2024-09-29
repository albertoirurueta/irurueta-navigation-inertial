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

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class StandardDeviationFrameBodyKinematicsTest {

    private static final double MIN_SPECIFIC_FORCE = -9.81;
    private static final double MAX_SPECIFIC_FORCE = 9.81;

    private static final double MIN_ANGULAR_RATE_VALUE = -1.0;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_TIME_INTERVAL = 0.01;
    private static final double MAX_TIME_INTERVAL = 0.03;

    private static final double THRESHOLD = 1e-6;

    @Test
    public void testConstructor() {
        // test empty constructor
        StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        Time timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        Acceleration acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        Acceleration acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        AngularSpeed angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with kinematics
        final BodyKinematics kinematics = new BodyKinematics();
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with ECEF frame
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with NED frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with time interval seconds
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeIntervalSeconds = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeIntervalSeconds);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(-1.0));

        // test constructor with time interval
        final Time timeInterval2 = new Time(timeIntervalSeconds, TimeUnit.SECOND);

        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeInterval2);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(timeInterval2, frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, timeInterval2);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        final Time wrongTimeInterval = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(wrongTimeInterval));

        // test constructor with current and previous ECEF frame
        final NEDFrame previousNedFrame = new NEDFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with current and previous NED frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with current and previous ECEF frame and time interval seconds
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame,
                timeIntervalSeconds);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, -1.0));

        // test constructor with current and previous ECEF frame and time interval
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame, timeInterval2);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(timeIntervalSeconds, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, wrongTimeInterval));

        // test constructor with current and previous NED frame and time interval seconds
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeIntervalSeconds);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(timeIntervalSeconds, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, -1.0));

        // test constructor with current and previous NED frame and time interval
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeInterval2);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(timeIntervalSeconds, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(), acceleration1);
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, wrongTimeInterval));

        // test constructor with body kinematics and ECEF frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with body kinematics and NED frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(), acceleration1);
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with body kinematics, current and previous ECEF frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with body kinematics, current and previous NED frame
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // test constructor with body kinematics, current and previous ECEF frame and
        // time interval seconds
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeIntervalSeconds);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(timeIntervalSeconds, TimeUnit.SECOND));
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, -1.0));

        // test constructor with body kinematics, current and previous ECEF frame and
        // time interval
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval2);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, wrongTimeInterval));

        // test constructor with body kinematics, current and previous NED frame and
        // time interval seconds
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeIntervalSeconds);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, -1.0));

        // test constructor with body kinematics, current and previous NED frame and
        // time interval seconds
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeInterval2);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, wrongTimeInterval));

        // test constructor with specific force and angular rate standard deviations
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                specificForceStandardDeviation, -1.0));

        // test constructor with ECEF frame, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                specificForceStandardDeviation, -1.0));

        // test constructor with NED frame, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                specificForceStandardDeviation, -1.0));

        // test constructor with time interval, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeIntervalSeconds,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(-1.0,
                specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeIntervalSeconds,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeIntervalSeconds,
                specificForceStandardDeviation, -1.0));

        // test constructor with time interval, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeInterval2, specificForceStandardDeviation,
                angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(timeInterval2, frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, timeInterval2);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(wrongTimeInterval,
                specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeInterval2,
                -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeInterval2,
                specificForceStandardDeviation, -1.0));

        // test constructor with current and previous ECEF frame, specific force and
        // angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, specificForceStandardDeviation, -1.0));

        // test constructor with current and previous NED frame, specific force and
        // angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, specificForceStandardDeviation, -1.0));

        // test constructor with current and previous ECEF frame, time interval seconds,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame,
                timeIntervalSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, -1.0, specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeIntervalSeconds, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeIntervalSeconds, specificForceStandardDeviation, -1.0));

        // test constructor with current and previous ECEF frame, time interval,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame, timeInterval2,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, wrongTimeInterval, specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeInterval2, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeInterval2, specificForceStandardDeviation, -1.0));

        // test constructor with current and previous NED frame, time interval seconds,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeIntervalSeconds,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, -1.0, specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeIntervalSeconds, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeIntervalSeconds, specificForceStandardDeviation, -1.0));

        // test constructor with current and previous NED frame, time interval,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeInterval2,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, wrongTimeInterval, specificForceStandardDeviation, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeInterval2, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeInterval2, specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics and ECEF frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics and NED frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics, current and previous ECEF frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics, current and previous NED frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, -1.0, angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, specificForceStandardDeviation, -1.0));

        // test constructor with body kinematics, current and previous ECEF frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeIntervalSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, -1.0, specificForceStandardDeviation,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeIntervalSeconds, -1.0,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeIntervalSeconds, specificForceStandardDeviation,
                -1.0));

        // test constructor with body kinematics, current and previous ECEF frame,
        // time interval, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval2, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, wrongTimeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeInterval2, -1.0,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeInterval2, specificForceStandardDeviation,
                -1.0));

        // test constructor with body kinematics, current and previous NED frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeIntervalSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, -1.0, specificForceStandardDeviation,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeIntervalSeconds, -1.0,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeIntervalSeconds, specificForceStandardDeviation,
                -1.0));

        // test constructor with body kinematics, current and previous NED frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeInterval2, specificForceStandardDeviation, angularRateStandardDeviation);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, wrongTimeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeInterval2, -1.0,
                angularRateStandardDeviation));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeInterval2, specificForceStandardDeviation,
                -1.0));

        // test constructor with specific force and angular rate standard deviations
        final Acceleration specificForceStandardDeviation1 = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed angularRateStandardDeviation1 = new AngularSpeed(angularRateStandardDeviation,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(specificForceStandardDeviation1,
                angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        final Acceleration wrongAcceleration = new Acceleration(-1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed wrongAngularRate = new AngularSpeed(-1.0, AngularSpeedUnit.RADIANS_PER_SECOND);

        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(wrongAcceleration,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, specificForceStandardDeviation1,
                angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with ECEF frame, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, specificForceStandardDeviation1,
                angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with NED frame, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, specificForceStandardDeviation1,
                angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with time interval, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeIntervalSeconds,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                -1.0, specificForceStandardDeviation1, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                timeIntervalSeconds, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(
                timeIntervalSeconds, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with time interval, specific force and angular rate
        // standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(timeInterval2, specificForceStandardDeviation1,
                angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(timeInterval2, frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, timeInterval2);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(wrongTimeInterval,
                specificForceStandardDeviation1, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeInterval2,
                wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(timeInterval2,
                specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous ECEF frame, specific force and
        // angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous NED frame, specific force and
        // angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous ECEF frame, time interval seconds,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame,
                timeIntervalSeconds, specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, -1.0, specificForceStandardDeviation1, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeIntervalSeconds, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeIntervalSeconds, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous ECEF frame, time interval,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(ecefFrame, previousEcefFrame, timeInterval2,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(timeIntervalSeconds, TimeUnit.SECOND));
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, wrongTimeInterval, specificForceStandardDeviation1,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeInterval2, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(ecefFrame,
                previousEcefFrame, timeInterval2, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous NED frame, time interval seconds,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeIntervalSeconds,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, -1.0, specificForceStandardDeviation1, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeIntervalSeconds, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeIntervalSeconds, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with current and previous NED frame, time interval,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(nedFrame, previousNedFrame, timeInterval2,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, wrongTimeInterval, specificForceStandardDeviation1, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeInterval2, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(nedFrame,
                previousNedFrame, timeInterval2, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics and ECEF frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics and NED frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous ECEF frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous NED frame,
        // specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(0.0, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous ECEF frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeIntervalSeconds, specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, -1.0, specificForceStandardDeviation1,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeIntervalSeconds, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeIntervalSeconds, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous ECEF frame,
        // time interval, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval2, specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, wrongTimeInterval, specificForceStandardDeviation1,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeInterval2, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                ecefFrame, previousEcefFrame, timeInterval2, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous NED frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeIntervalSeconds, specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, -1.0, specificForceStandardDeviation1,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeIntervalSeconds, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeIntervalSeconds, specificForceStandardDeviation1, wrongAngularRate));

        // test constructor with body kinematics, current and previous NED frame,
        // time interval seconds, specific force and angular rate standard deviations
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeInterval2, specificForceStandardDeviation1, angularRateStandardDeviation1);

        // check default values
        assertSame(kinematics, frameBodyKinematics.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, wrongTimeInterval, specificForceStandardDeviation1,
                angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeInterval2, wrongAcceleration, angularRateStandardDeviation1));
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyKinematics(kinematics,
                nedFrame, previousNedFrame, timeInterval2, specificForceStandardDeviation1, wrongAngularRate));

        // test copy constructor
        frameBodyKinematics = new StandardDeviationFrameBodyKinematics(kinematics, nedFrame, previousNedFrame,
                timeIntervalSeconds, specificForceStandardDeviation, angularRateStandardDeviation);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                frameBodyKinematics);

        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
        assertEquals(nedFrame, frameBodyKinematics2.getNedFrame());
        assertEquals(previousEcefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(previousNedFrame, frameBodyKinematics.getPreviousNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyKinematics2.getPreviousNedFrame(nedFrame2));
        assertEquals(previousNedFrame, nedFrame2);
        assertEquals(timeIntervalSeconds, frameBodyKinematics2.getTimeInterval(), 0.0);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), frameBodyKinematics2.getTimeIntervalAsTime());
        timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics2.getTimeIntervalAsTime(timeInterval);
        assertEquals(new Time(timeIntervalSeconds, TimeUnit.SECOND), timeInterval);
        assertEquals(specificForceStandardDeviation, frameBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        acceleration1 = new Acceleration(specificForceStandardDeviation, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(acceleration1, frameBodyKinematics2.getSpecificForceStandardDeviationAsAcceleration());
        acceleration2 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics2.getSpecificForceStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(angularRateStandardDeviation, frameBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
        angularSpeed1 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, frameBodyKinematics2.getAngularRateStandardDeviationAsAngularSpeed());
        angularSpeed2 = new AngularSpeed(0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        frameBodyKinematics2.getAngularRateStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
    }

    @Test
    public void testGetSetSpecificForceStandardDeviation() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertEquals(0.0, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);

        frameBodyKinematics.setSpecificForceStandardDeviation(specificForceStandardDeviation);

        // check
        assertEquals(specificForceStandardDeviation, frameBodyKinematics.getSpecificForceStandardDeviation(), 0.0);
    }

    @Test
    public void testGetSetSpecificForceStandardDeviationAsAcceleration() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        Acceleration value1 = frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration();
        assertEquals(0.0, value1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, value1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final Acceleration value2 = new Acceleration(specificForceStandardDeviation,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        frameBodyKinematics.setSpecificForceStandardDeviation(value2);

        // check
        final Acceleration value3 = frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration();
        final Acceleration value4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        frameBodyKinematics.getSpecificForceStandardDeviationAsAcceleration(value4);

        assertEquals(value2, value3);
        assertEquals(value2, value4);
    }

    @Test
    public void testGetSetAngularRateStandardDeviation() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertEquals(0.0, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        frameBodyKinematics.setAngularRateStandardDeviation(angularRateStandardDeviation);

        // check
        assertEquals(angularRateStandardDeviation, frameBodyKinematics.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testGetSetAngularRateStandardDeviationAsAngularSpeed() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        AngularSpeed value1 = frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        assertEquals(0.0, value1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, value1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final AngularSpeed value2 = new AngularSpeed(angularRateStandardDeviation, AngularSpeedUnit.RADIANS_PER_SECOND);

        frameBodyKinematics.setAngularRateStandardDeviation(value2);

        // check
        final AngularSpeed value3 = frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed();
        final AngularSpeed value4 = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        frameBodyKinematics.getAngularRateStandardDeviationAsAngularSpeed(value4);

        assertEquals(value2, value3);
        assertEquals(value2, value4);
    }

    @Test
    public void testGetSetKinematics() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getKinematics());

        // set new value
        final BodyKinematics bodyKinematics = new BodyKinematics();
        frameBodyKinematics.setKinematics(bodyKinematics);

        // check
        assertSame(bodyKinematics, frameBodyKinematics.getKinematics());
    }

    @Test
    public void testGetSetFrame() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setFrame(ecefFrame);

        // check
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        final NEDFrame nedFrame2 = new NEDFrame();
        frameBodyKinematics.getNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    public void testGetSetNedFrame() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        final NEDFrame nedFrame2 = new NEDFrame();
        frameBodyKinematics.getNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    public void testGetSetPreviousFrame() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setPreviousFrame(ecefFrame);

        // check
        assertSame(ecefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(nedFrame, frameBodyKinematics.getPreviousNedFrame());
        final NEDFrame nedFrame2 = new NEDFrame();
        frameBodyKinematics.getPreviousNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    public void testGetSetPreviousNedFrame() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics =
                new StandardDeviationFrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setPreviousNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(nedFrame, frameBodyKinematics.getPreviousNedFrame());
        final NEDFrame nedFrame2 = new NEDFrame();
        frameBodyKinematics.getPreviousNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    public void testGetSetTimeInterval() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        frameBodyKinematics.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, frameBodyKinematics.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> frameBodyKinematics.setTimeInterval(-1.0));
    }

    @Test
    public void testGetSetTimeIntervalAsTime() {
        final StandardDeviationFrameBodyKinematics frameBodyKinematics = new StandardDeviationFrameBodyKinematics();

        // check default value
        final Time timeInterval1 = frameBodyKinematics.getTimeIntervalAsTime();
        assertEquals(0.0, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Time timeInterval2 = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL),
                TimeUnit.SECOND);
        frameBodyKinematics.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = frameBodyKinematics.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    public void testCopyFromWhenBodyKinematicsAndFrameAreAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();

        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    public void testCopyFromWhenEmptySourceAndDestinationHasKinematicsCurrentAndPreviousFrameAndTimeInterval()
            throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics();
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertNull(frameBodyKinematics2.getFrame());
        assertNull(frameBodyKinematics2.getPreviousFrame());
        assertEquals(0.0, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    public void testCopyFromWhenSourceAndDestinationHaveKinematicsCurrentAndPreviousFrameAndTimeInterval()
            throws InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final BodyKinematics kinematics1 = createBodyKinematics();
        final NEDFrame nedFrame1 = createNedFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final NEDFrame previousNedFrame1 = createNedFrame();
        final ECEFFrame previousEcefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame1);
        final double timeInterval1 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final BodyKinematics kinematics2 = createBodyKinematics();
        final NEDFrame nedFrame2 = createNedFrame();
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final NEDFrame previousNedFrame2 = createNedFrame();
        final ECEFFrame previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final double timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics1, ecefFrame1, previousEcefFrame1, timeInterval1, specificForceStandardDeviation1,
                angularRateStandardDeviation1);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics2, ecefFrame2, previousEcefFrame2, timeInterval2, specificForceStandardDeviation2,
                angularRateStandardDeviation2);

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics1, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame1, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame1, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval1, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    public void testCopyTo() throws InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final BodyKinematics kinematics1 = createBodyKinematics();
        final NEDFrame nedFrame1 = createNedFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final NEDFrame previousNedFrame1 = createNedFrame();
        final ECEFFrame previousEcefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame1);
        final double timeInterval1 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation1 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation1 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final BodyKinematics kinematics2 = createBodyKinematics();
        final NEDFrame nedFrame2 = createNedFrame();
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final NEDFrame previousNedFrame2 = createNedFrame();
        final ECEFFrame previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final double timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation2 = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation2 = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics1, ecefFrame1, previousEcefFrame1, timeInterval1, specificForceStandardDeviation1,
                angularRateStandardDeviation1);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics2, ecefFrame2, previousEcefFrame2, timeInterval2, specificForceStandardDeviation2,
                angularRateStandardDeviation2);

        frameBodyKinematics1.copyTo(frameBodyKinematics2);

        // check
        assertEquals(kinematics1, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame1, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame1, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval1, frameBodyKinematics2.getTimeInterval(), 0.0);
        assertEquals(specificForceStandardDeviation1, frameBodyKinematics2.getSpecificForceStandardDeviation(), 0.0);
        assertEquals(angularRateStandardDeviation1, frameBodyKinematics2.getAngularRateStandardDeviation(), 0.0);
    }

    @Test
    public void testHashCode() throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics3 = new StandardDeviationFrameBodyKinematics();

        assertEquals(frameBodyKinematics1.hashCode(), frameBodyKinematics2.hashCode());
        assertNotEquals(frameBodyKinematics1.hashCode(), frameBodyKinematics3.hashCode());
    }

    @Test
    public void testEquals() throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics3 = new StandardDeviationFrameBodyKinematics();

        //noinspection EqualsWithItself
        assertEquals(frameBodyKinematics1, frameBodyKinematics1);
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics2));
        assertFalse(frameBodyKinematics1.equals(frameBodyKinematics3));
        assertNotEquals(frameBodyKinematics1, null);
        assertNotEquals(frameBodyKinematics1, new Object());
    }

    @Test
    public void testEqualsWithThreshold() throws InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics3 = new StandardDeviationFrameBodyKinematics();

        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics1, THRESHOLD));
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics2, THRESHOLD));
        assertFalse(frameBodyKinematics1.equals(frameBodyKinematics3, THRESHOLD));
        assertFalse(frameBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    public void testClone() throws InvalidSourceAndDestinationFrameTypeException, CloneNotSupportedException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);

        final Object frameBodyKinematics2 = frameBodyKinematics1.clone();

        // check
        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    public void testSerializeDeserialize() throws InvalidSourceAndDestinationFrameTypeException, IOException,
            ClassNotFoundException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final double specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final double angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);

        final StandardDeviationFrameBodyKinematics frameBodyKinematics1 = new StandardDeviationFrameBodyKinematics(
                kinematics, ecefFrame, previousEcefFrame, timeInterval, specificForceStandardDeviation,
                angularRateStandardDeviation);

        final byte[] bytes = SerializationHelper.serialize(frameBodyKinematics1);
        final StandardDeviationFrameBodyKinematics frameBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
        assertNotSame(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = StandardDeviationFrameBodyKinematics.class.getDeclaredField("serialVersionUID");
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

    private static NEDFrame createNedFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }
}
