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
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;

import static org.junit.jupiter.api.Assertions.*;

class FrameBodyKinematicsTest {

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
    void testConstructor() {
        // test empty constructor
        var frameBodyKinematics = new FrameBodyKinematics();

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

        // test constructor with kinematics
        final var kinematics = new BodyKinematics();
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
        assertEquals(new Time(0.0, TimeUnit.SECOND), timeInterval);

        // test constructor with ECEF frame
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyKinematics = new FrameBodyKinematics(ecefFrame);

        // check default values
        assertNull(frameBodyKinematics.getKinematics());
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        var nedFrame2 = new NEDFrame();
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
        final var randomizer = new UniformRandomizer();
        final var timeIntervalSeconds = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

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

        // test constructor with a time interval
        final var timeInterval2 = new Time(timeIntervalSeconds, TimeUnit.SECOND);

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
        final var wrongTimeInterval = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> new FrameBodyKinematics(wrongTimeInterval));

        // test constructor with current and previous ECEF frame
        final var previousNedFrame = new NEDFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

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

        final var frameBodyKinematics2 = new FrameBodyKinematics(frameBodyKinematics);

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
    void testGetSetKinematics() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getKinematics());

        // set new value
        final var bodyKinematics = new BodyKinematics();
        frameBodyKinematics.setKinematics(bodyKinematics);

        // check
        assertSame(bodyKinematics, frameBodyKinematics.getKinematics());
    }

    @Test
    void testGetSetFrame() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getFrame());
        assertNull(frameBodyKinematics.getNedFrame());

        // set new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setFrame(ecefFrame);

        // check
        assertSame(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        final var nedFrame2 = new NEDFrame();
        frameBodyKinematics.getNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    void testGetSetNedFrame() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getNedFrame());
        assertFalse(frameBodyKinematics.getNedFrame(null));

        // set new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, frameBodyKinematics.getFrame());
        assertEquals(nedFrame, frameBodyKinematics.getNedFrame());
        final var nedFrame2 = new NEDFrame();
        frameBodyKinematics.getNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    void testGetSetPreviousFrame() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getPreviousFrame());
        assertNull(frameBodyKinematics.getPreviousNedFrame());

        // set new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setPreviousFrame(ecefFrame);

        // check
        assertSame(ecefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(nedFrame, frameBodyKinematics.getPreviousNedFrame());
        final var nedFrame2 = new NEDFrame();
        frameBodyKinematics.getPreviousNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    void testGetSetPreviousNedFrame() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertNull(frameBodyKinematics.getPreviousNedFrame());
        assertFalse(frameBodyKinematics.getPreviousNedFrame(null));

        // set new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        frameBodyKinematics.setPreviousNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, frameBodyKinematics.getPreviousFrame());
        assertEquals(nedFrame, frameBodyKinematics.getPreviousNedFrame());
        final var nedFrame2 = new NEDFrame();
        frameBodyKinematics.getPreviousNedFrame(nedFrame2);
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    void testGetSetTimeInterval() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        assertEquals(0.0, frameBodyKinematics.getTimeInterval(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        frameBodyKinematics.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, frameBodyKinematics.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> frameBodyKinematics.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() {
        final var frameBodyKinematics = new FrameBodyKinematics();

        // check default value
        final var timeInterval1 = frameBodyKinematics.getTimeIntervalAsTime();
        assertEquals(0.0, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var timeInterval2 = new Time(randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL),
                TimeUnit.SECOND);
        frameBodyKinematics.setTimeInterval(timeInterval2);

        // check
        final var timeInterval3 = frameBodyKinematics.getTimeIntervalAsTime();
        final var timeInterval4 = new Time(0.0, TimeUnit.MILLISECOND);
        frameBodyKinematics.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    void testCopyFromWhenBodyKinematicsAndFrameAreAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();

        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    void testCopyFromWhenOnlyBodyKinematicsAreAvailableAtSourceAndDestinationIsEmpty() {
        final var kinematics = createBodyKinematics();

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics);
        final var frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics, frameBodyKinematics2.getKinematics());
        assertNull(frameBodyKinematics2.getFrame());
    }

    @Test
    void testCopyFromWhenOnlyFrameIsAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyKinematics1 = new FrameBodyKinematics(ecefFrame);
        final var frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
    }

    @Test
    void testCopyFromWhenOnlyCurrentAndPreviousFrameIsAvailableAtSourceAndDestinationIsEmpty()
            throws InvalidSourceAndDestinationFrameTypeException {
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var frameBodyKinematics1 = new FrameBodyKinematics(ecefFrame, previousEcefFrame);
        final var frameBodyKinematics2 = new FrameBodyKinematics();

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame, frameBodyKinematics2.getFrame());
    }

    @Test
    void testCopyFromWhenEmptySourceAndDestinationHasKinematicsCurrentAndPreviousFrameAndTimeInterval()
            throws InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics();
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertNull(frameBodyKinematics2.getKinematics());
        assertNull(frameBodyKinematics2.getFrame());
        assertNull(frameBodyKinematics2.getPreviousFrame());
        assertEquals(0.0, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    void testCopyFromWhenSourceAndDestinationHaveKinematicsCurrentAndPreviousFrameAndTimeInterval()
            throws InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();

        final var kinematics1 = createBodyKinematics();
        final var nedFrame1 = createNedFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var previousNedFrame1 = createNedFrame();
        final var previousEcefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame1);
        final var timeInterval1 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var kinematics2 = createBodyKinematics();
        final var nedFrame2 = createNedFrame();
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var previousNedFrame2 = createNedFrame();
        final var previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final var timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics1, ecefFrame1, previousEcefFrame1,
                timeInterval1);
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics2, ecefFrame2, previousEcefFrame2,
                timeInterval2);

        frameBodyKinematics2.copyFrom(frameBodyKinematics1);

        // check
        assertEquals(kinematics1, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame1, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame1, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval1, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    void testCopyTo() throws InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();

        final var kinematics1 = createBodyKinematics();
        final var nedFrame1 = createNedFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var previousNedFrame1 = createNedFrame();
        final var previousEcefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame1);
        final var timeInterval1 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var kinematics2 = createBodyKinematics();
        final var nedFrame2 = createNedFrame();
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);
        final var previousNedFrame2 = createNedFrame();
        final var previousEcefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame2);
        final var timeInterval2 = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics1, ecefFrame1, previousEcefFrame1,
                timeInterval1);
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics2, ecefFrame2, previousEcefFrame2,
                timeInterval2);

        frameBodyKinematics1.copyTo(frameBodyKinematics2);

        // check
        assertEquals(kinematics1, frameBodyKinematics2.getKinematics());
        assertEquals(ecefFrame1, frameBodyKinematics2.getFrame());
        assertEquals(previousEcefFrame1, frameBodyKinematics2.getPreviousFrame());
        assertEquals(timeInterval1, frameBodyKinematics2.getTimeInterval(), 0.0);
    }

    @Test
    void testHashCode() throws InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics3 = new FrameBodyKinematics();

        assertEquals(frameBodyKinematics1.hashCode(), frameBodyKinematics2.hashCode());
        assertNotEquals(frameBodyKinematics1.hashCode(), frameBodyKinematics3.hashCode());
    }

    @Test
    void testEquals() throws InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics3 = new FrameBodyKinematics();

        //noinspection ConstantConditions,SimplifiableJUnitAssertion
        assertTrue(frameBodyKinematics1.equals((Object) frameBodyKinematics1));
        //noinspection EqualsWithItself
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics1));
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics2));
        assertFalse(frameBodyKinematics1.equals(frameBodyKinematics3));
        //noinspection SimplifiableJUnitAssertion,ConstantConditions
        assertNotEquals(null, frameBodyKinematics1);
        assertFalse(frameBodyKinematics1.equals(null));
        //noinspection SimplifiableJUnitAssertion
        assertNotEquals(new Object(), frameBodyKinematics1);
    }

    @Test
    void testEqualsWithThreshold() throws InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics2 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);
        final var frameBodyKinematics3 = new FrameBodyKinematics();

        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics1, THRESHOLD));
        assertTrue(frameBodyKinematics1.equals(frameBodyKinematics2, THRESHOLD));
        assertFalse(frameBodyKinematics1.equals(frameBodyKinematics3, THRESHOLD));
        assertFalse(frameBodyKinematics1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws InvalidSourceAndDestinationFrameTypeException, CloneNotSupportedException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);

        final var frameBodyKinematics2 = frameBodyKinematics1.clone();

        // check
        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException,
            InvalidSourceAndDestinationFrameTypeException {
        final var kinematics = createBodyKinematics();
        final var nedFrame = createNedFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var previousNedFrame = createNedFrame();
        final var previousEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(previousNedFrame);

        final var randomizer = new UniformRandomizer();
        final var timeInterval = randomizer.nextDouble(MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);

        final var frameBodyKinematics1 = new FrameBodyKinematics(kinematics, ecefFrame, previousEcefFrame,
                timeInterval);

        final var bytes = SerializationHelper.serialize(frameBodyKinematics1);
        final var frameBodyKinematics2 = SerializationHelper.deserialize(bytes);

        assertEquals(frameBodyKinematics1, frameBodyKinematics2);
        assertNotSame(frameBodyKinematics1, frameBodyKinematics2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = FrameBodyKinematics.class.getDeclaredField("serialVersionUID");
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

    private static NEDFrame createNedFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var roll = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toDegrees(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var c = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }
}
