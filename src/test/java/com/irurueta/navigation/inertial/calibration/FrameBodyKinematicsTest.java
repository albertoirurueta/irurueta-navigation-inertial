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
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Random;

import static org.junit.Assert.*;

public class FrameBodyKinematicsTest {

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
        FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);
        assertEquals(frameBodyKinematics.getTimeIntervalAsTime(), new Time(0.0, TimeUnit.SECOND));
        Time timeInterval = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval);
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));

        // test constructor with kinematics
        final BodyKinematics kinematics = new BodyKinematics();
        frameBodyKinematics = new FrameBodyKinematics(kinematics);

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
        assertEquals(timeInterval, new Time(0.0, TimeUnit.SECOND));

        // test constructor with ECEF frame
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyKinematics = new FrameBodyKinematics(ecefFrame);

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

        // test constructor with NED frame
        frameBodyKinematics = new FrameBodyKinematics(nedFrame);

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

        // test constructor with time interval seconds
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeIntervalSeconds = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        frameBodyKinematics = new FrameBodyKinematics(timeIntervalSeconds);
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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(-1.0));

        // test constructor with time interval
        final Time timeInterval2 = new Time(timeIntervalSeconds, TimeUnit.SECOND);

        frameBodyKinematics = new FrameBodyKinematics(timeInterval2);
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

        // Force IllegalArgumentException
        final Time wrongTimeInterval = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(wrongTimeInterval));

        // test constructor with current and previous ECEF frame
        final NEDFrame previousNedFrame = new NEDFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        frameBodyKinematics = new FrameBodyKinematics(ecefFrame, previousEcefFrame);

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

        // test constructor with current and previous NED frame
        frameBodyKinematics = new FrameBodyKinematics(nedFrame, previousNedFrame);

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

        // test constructor with current and previous ECEF frame and time interval seconds
        frameBodyKinematics = new FrameBodyKinematics(ecefFrame, previousEcefFrame, timeIntervalSeconds);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(ecefFrame, previousEcefFrame,
                -1.0));

        // test constructor with current and previous ECEF frame and time interval
        frameBodyKinematics = new FrameBodyKinematics(ecefFrame, previousEcefFrame, timeInterval2);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(ecefFrame, previousEcefFrame,
                wrongTimeInterval));

        // test constructor with current and previous NED frame and time interval seconds
        frameBodyKinematics = new FrameBodyKinematics(nedFrame, previousNedFrame, timeIntervalSeconds);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(nedFrame, previousNedFrame,
                -1.0));

        // test constructor with current and previous NED frame and time interval
        frameBodyKinematics = new FrameBodyKinematics(nedFrame, previousNedFrame, timeInterval2);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(nedFrame, previousNedFrame,
                wrongTimeInterval));

        // test constructor with body kinematics and ECEF frame
        frameBodyKinematics = new FrameBodyKinematics(kinematics, ecefFrame);

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

        // test constructor with body kinematics and NED frame
        frameBodyKinematics = new FrameBodyKinematics(kinematics, nedFrame);

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

        // test constructor with body kinematics, current and previous ECEF frame
        frameBodyKinematics = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame);

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

        // test constructor with body kinematics, current and previous NED frame
        frameBodyKinematics = new FrameBodyKinematics(kinematics, nedFrame, previousNedFrame);

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

        // test constructor with body kinematics, current and previous ECEF frame and
        // time interval seconds
        frameBodyKinematics = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame, timeIntervalSeconds);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, -1.0));

        // test constructor with body kinematics, current and previous ECEF frame and
        // time interval
        frameBodyKinematics = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame, timeInterval2);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, wrongTimeInterval));

        // test constructor with body kinematics, current and previous NED frame and
        // time interval seconds
        frameBodyKinematics = new FrameBodyKinematics(kinematics, nedFrame, previousNedFrame, timeIntervalSeconds);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(kinematics, nedFrame,
                previousNedFrame, -1.0));

        // test constructor with body kinematics, current and previous NED frame and
        // time interval seconds
        frameBodyKinematics = new FrameBodyKinematics(kinematics, nedFrame, previousNedFrame, timeInterval2);

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

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(kinematics, nedFrame,
                previousNedFrame, wrongTimeInterval));

        // test copy constructor
        frameBodyKinematics = new FrameBodyKinematics(kinematics, nedFrame, previousNedFrame, timeIntervalSeconds);

        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(frameBodyKinematics);

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
    }

    @Test
    public void testGetSetKinematics() {
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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
        final FrameBodyKinematics frameBodyKinematics = new FrameBodyKinematics();

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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    public void testCopyFromWhenOnlyBodyKinematicsAreAvailableAtSourceAndDestinationIsEmpty() {
        final BodyKinematics kinematics = createBodyKinematics();

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertNull(frameBodyKinematics2.getFrame());
    }

    @Test
    public void testCopyFromWhenOnlyFrameIsAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(ecefFrame);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
    }

    @Test
    public void testCopyFromWhenOnlyCurrentAndPreviousFrameIsAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(ecefFrame, previousEcefFrame);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics();
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);

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

        final BodyKinematics kinematics2 = createBodyKinematics();
        final NEDFrame nedFrame2 = createNedFrame();
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final NEDFrame previousNedFrame2 = createNedFrame();
        final ECEFFrame previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final double timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics1, ecefFrame1,
                previousEcefFrame1, timeInterval1);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics2, ecefFrame2,
                previousEcefFrame2, timeInterval2);

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

        final BodyKinematics kinematics2 = createBodyKinematics();
        final NEDFrame nedFrame2 = createNedFrame();
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final NEDFrame previousNedFrame2 = createNedFrame();
        final ECEFFrame previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final double timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics1, ecefFrame1,
                previousEcefFrame1, timeInterval1);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics2, ecefFrame2,
                previousEcefFrame2, timeInterval2);

        frameBodyKinematics1.copyTo(frameBodyKinematics2);

        // check
        assertEquals(kinematics1, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame1, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame1, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval1, frameBodyKinematics2.getTimeInterval(), 0.0);
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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics3 = new FrameBodyKinematics();

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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics3 = new FrameBodyKinematics();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frameBodyKinematics1.equals((Object) frameBodyKinematics1));
        //noinspection EqualsWithItself
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics1));
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics2));
        assertFalse(frameBodyKinematics1.equals(frameBodyKinematics3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertNotEquals(frameBodyKinematics1, null);
        assertFalse(frameBodyKinematics1.equals(null));
        //noinspection SimplifiableJUnitAssertion
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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);
        final FrameBodyKinematics frameBodyKinematics3 = new FrameBodyKinematics();

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

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);

        final Object frameBodyKinematics2 = frameBodyKinematics1.clone();

        // check
        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException,
            InvalidSourceAndDestinationFrameTypeException {
        final BodyKinematics kinematics = createBodyKinematics();
        final NEDFrame nedFrame = createNedFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDFrame previousNedFrame = createNedFrame();
        final ECEFFrame previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final FrameBodyKinematics frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame,
                previousEcefFrame, timeInterval);

        final byte[] bytes = SerializationHelper.serialize(frameBodyKinematics1);
        final FrameBodyKinematics frameBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
        assertNotSame(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    public void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final Field field = FrameBodyKinematics.class.getDeclaredField("serialVersionUID");
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
