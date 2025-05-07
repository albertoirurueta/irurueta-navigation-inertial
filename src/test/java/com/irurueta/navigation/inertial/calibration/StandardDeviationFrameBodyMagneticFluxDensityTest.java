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
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.SerializationHelper;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import static org.junit.jupiter.api.Assertions.*;

class StandardDeviationFrameBodyMagneticFluxDensityTest {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAX_MAGNETIC_FLUX_DENSITY = 70e-6;

    private static final double THRESHOLD = 1e-6;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    void testConstructor() throws IOException {
        // test empty constructor
        var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density
        final var magneticFluxDensity = createMagneticFluxDensity();
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with ECEF frame
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        var nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with NED frame
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density and ECEF frame
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, 
                ecefFrame);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density and NED frame
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        final var timestamp = createTimestamp();
        final var calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final var year = FrameBodyMagneticFluxDensity.convertTime(timestamp);

        // test constructor with year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density and year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, year);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with ECEF frame and year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with NED frame and year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, year);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, ECEF frame and year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                year);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, NED frame and year
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                year);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density and date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, 
                timestamp);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with ECEF frame and date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with NED frame and date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, timestamp);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, ECEF frame and date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                timestamp);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, NED frame and date
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                timestamp);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density and calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, calendar);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with ECEF frame and calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with NED frame and calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, calendar);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, ECEF frame and
        // calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                calendar);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // test constructor with magnetic flux density, NED frame and
        // calendar
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                calendar);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        // test constructor with year and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(year, 
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 
                magneticFluxDensityStandardDeviation, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(year,
                -1.0));

        // test constructor with magnetic flux density, year and standard
        // deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, year,
                magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, year, -1.0));

        // test constructor with ECEF frame, year and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, year,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                ecefFrame, year, -1.0));

        // test constructor with NED frame, year and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, year,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, 
                year, -1.0));

        // test constructor with magnetic flux density, ECEF frame, year
        // and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                year, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, ecefFrame, year, -1.0));

        // test constructor with magnetic flux density, NED frame, year
        // and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                year, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, nedFrame, year, -1.0));

        // test constructor with date and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(timestamp,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(timestamp,
                -1.0));

        // test constructor with magnetic flux density, date and standard
        // deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, timestamp,
                magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, timestamp, -1.0));

        // test constructor with ECEF frame, date and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, timestamp,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame,
                timestamp, -1.0));

        // test constructor with NED frame, date and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, timestamp,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame,
                timestamp, -1.0));

        // test constructor with magnetic flux density, ECEF frame, date
        // and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                timestamp, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, ecefFrame, timestamp, -1.0));

        // test constructor with magnetic flux density, NED frame, date
        // and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                timestamp, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, nedFrame, timestamp, -1.0));

        // test constructor with calendar and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(calendar,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(calendar,
                -1.0));

        // test constructor with magnetic flux density, calendar and
        // standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, calendar,
                magneticFluxDensityStandardDeviation);

        // check default value
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity.getFrame());
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, calendar, -1.0));

        // test constructor with ECEF frame, calendar and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame, calendar,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame,
                calendar, -1.0));

        // test constructor with NED frame, calendar and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame, calendar,
                magneticFluxDensityStandardDeviation);

        // check default values
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(nedFrame,
                calendar, -1.0));

        // test constructor with magnetic flux density, ECEF frame,
        // calendar and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, ecefFrame,
                calendar, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertSame(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation,
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, ecefFrame, calendar, -1.0));

        // test constructor with magnetic flux density, NED frame,
        // calendar and standard deviation
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                calendar, magneticFluxDensityStandardDeviation);

        // check default values
        assertSame(magneticFluxDensity, frameBodyMagneticFluxDensity.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new StandardDeviationFrameBodyMagneticFluxDensity(
                magneticFluxDensity, nedFrame, calendar, -1.0));

        // test copy constructor
        frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity(magneticFluxDensity, nedFrame,
                calendar, magneticFluxDensityStandardDeviation);

        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                frameBodyMagneticFluxDensity);

        // check default values
        assertEquals(magneticFluxDensity, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(nedFrame, frameBodyMagneticFluxDensity2.getNedFrame());
        nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity2.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
        assertEquals(year, frameBodyMagneticFluxDensity2.getYear(), 0.0);
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetMagneticFluxDensityStandardDeviation() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        frameBodyMagneticFluxDensity.setMagneticFluxDensityStandardDeviation(magneticFluxDensityStandardDeviation);

        // check
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testGetSetMagneticFluxDensity() throws IOException {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getMagneticFluxDensity());

        // set a new value
        final var magneticFluxDensity = createMagneticFluxDensity();
        frameBodyMagneticFluxDensity.setMagneticFluxDensity(magneticFluxDensity);

        // check
        assertSame(frameBodyMagneticFluxDensity.getMagneticFluxDensity(), magneticFluxDensity);
    }

    @Test
    void testGetSetFrame() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getFrame());

        // set a new value
        final var frame = new ECEFFrame();
        frameBodyMagneticFluxDensity.setFrame(frame);

        // check
        assertSame(frame, frameBodyMagneticFluxDensity.getFrame());
    }

    @Test
    void testGetSetNedFrame() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertNull(frameBodyMagneticFluxDensity.getNedFrame());
        assertFalse(frameBodyMagneticFluxDensity.getNedFrame(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        frameBodyMagneticFluxDensity.setNedFrame(nedFrame);

        // check
        assertEquals(nedFrame, frameBodyMagneticFluxDensity.getNedFrame());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity.getFrame());
        final var nedFrame2 = new NEDFrame();
        assertTrue(frameBodyMagneticFluxDensity.getNedFrame(nedFrame2));
        assertEquals(nedFrame, nedFrame2);
    }

    @Test
    void testGetSetYear() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);

        // set a new value
        final var timestamp = createTimestamp();
        final var year = FrameBodyMagneticFluxDensity.convertTime(timestamp);

        frameBodyMagneticFluxDensity.setYear(year);

        // check
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
    }

    @Test
    void testSetTime1() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);

        // set a new value
        final var timestamp = createTimestamp();
        final var year = FrameBodyMagneticFluxDensity.convertTime(timestamp);

        frameBodyMagneticFluxDensity.setTime(timestamp);

        // check
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
    }

    @Test
    void testSetTime2() {
        final var frameBodyMagneticFluxDensity = new StandardDeviationFrameBodyMagneticFluxDensity();

        // check default value
        assertEquals(0.0, frameBodyMagneticFluxDensity.getYear(), 0.0);

        // set a new value
        final var timestamp = createTimestamp();
        final var calendar = new GregorianCalendar();
        calendar.setTime(timestamp);
        final var year = FrameBodyMagneticFluxDensity.convertTime(timestamp);

        frameBodyMagneticFluxDensity.setTime(calendar);

        // check
        assertEquals(year, frameBodyMagneticFluxDensity.getYear(), 0.0);
    }

    @Test
    void testCopyFromWhenBodyMagneticFluxAndFrameAreAvailableAtSourceAndDestinationIsEmpty() throws IOException,
            InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertEquals(bodyMagneticFluxDensity, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenOnlyBodyMagneticFluxAreaAvailableAtSourceAndDestinationIsEmpty() throws IOException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertEquals(bodyMagneticFluxDensity, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(magneticFluxDensityStandardDeviation, 
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenOnlyFrameIsAvailableAtSourceAndDestinationIsEmpty() 
            throws InvalidSourceAndDestinationFrameTypeException {

        final var position = createPosition();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(ecefFrame);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity();

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertNull(frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame, frameBodyMagneticFluxDensity2.getFrame());
    }

    @Test
    void testCopyFromWhenEmptySourceAndDestinationHasData() throws IOException, 
            InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity();
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        // check
        assertNull(frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertNull(frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(0.0, frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyFromWhenBothSourceAndDestinationHaveData() throws IOException, 
            InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation1 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensityStandardDeviation2 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position1 = createPosition();
        final var position2 = createPosition();
        final var timestamp1 = createTimestamp();
        final var timestamp2 = createTimestamp();
        final var cnb1 = createAttitude();
        final var cnb2 = createAttitude();
        final var cbn1 = cnb1.inverseAndReturnNew();
        final var cbn2 = cnb1.inverseAndReturnNew();

        final var bodyMagneticFluxDensity1 = createMagneticFluxDensity(position1, timestamp1, cnb1);
        final var bodyMagneticFluxDensity2 = createMagneticFluxDensity(position2, timestamp2, cnb2);

        final var nedFrame1 = new NEDFrame(position1, cbn1);
        final var nedFrame2 = new NEDFrame(position2, cbn2);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity1, ecefFrame1, timestamp1, magneticFluxDensityStandardDeviation1);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity2, ecefFrame2, timestamp2, magneticFluxDensityStandardDeviation2);

        frameBodyMagneticFluxDensity2.copyFrom(frameBodyMagneticFluxDensity1);

        assertEquals(bodyMagneticFluxDensity1, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame1, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(frameBodyMagneticFluxDensity1.getYear(), frameBodyMagneticFluxDensity2.getYear(), 0.0);
        assertEquals(frameBodyMagneticFluxDensity1.getMagneticFluxDensityStandardDeviation(),
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testCopyTo() throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation1 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);
        final var magneticFluxDensityStandardDeviation2 = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position1 = createPosition();
        final var position2 = createPosition();
        final var timestamp1 = createTimestamp();
        final var timestamp2 = createTimestamp();
        final var cnb1 = createAttitude();
        final var cnb2 = createAttitude();
        final var cbn1 = cnb1.inverseAndReturnNew();
        final var cbn2 = cnb1.inverseAndReturnNew();

        final var bodyMagneticFluxDensity1 = createMagneticFluxDensity(position1, timestamp1, cnb1);
        final var bodyMagneticFluxDensity2 = createMagneticFluxDensity(position2, timestamp2, cnb2);

        final var nedFrame1 = new NEDFrame(position1, cbn1);
        final var nedFrame2 = new NEDFrame(position2, cbn2);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity1, ecefFrame1, timestamp1, magneticFluxDensityStandardDeviation1);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity2, ecefFrame2, timestamp2, magneticFluxDensityStandardDeviation2);

        frameBodyMagneticFluxDensity1.copyTo(frameBodyMagneticFluxDensity2);

        // check
        assertEquals(bodyMagneticFluxDensity1, frameBodyMagneticFluxDensity2.getMagneticFluxDensity());
        assertEquals(ecefFrame1, frameBodyMagneticFluxDensity2.getFrame());
        assertEquals(frameBodyMagneticFluxDensity1.getYear(), frameBodyMagneticFluxDensity2.getYear(), 0.0);
        assertEquals(frameBodyMagneticFluxDensity1.getMagneticFluxDensityStandardDeviation(),
                frameBodyMagneticFluxDensity2.getMagneticFluxDensityStandardDeviation(), 0.0);
    }

    @Test
    void testHashCode() throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity3 = new StandardDeviationFrameBodyMagneticFluxDensity();

        assertEquals(frameBodyMagneticFluxDensity1.hashCode(), frameBodyMagneticFluxDensity2.hashCode());
        assertNotEquals(frameBodyMagneticFluxDensity1.hashCode(), frameBodyMagneticFluxDensity3.hashCode());
    }

    @Test
    void testEquals() throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity3 = new StandardDeviationFrameBodyMagneticFluxDensity();

        //noinspection EqualsWithItself
        assertEquals(frameBodyMagneticFluxDensity1, frameBodyMagneticFluxDensity1);
        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity2));
        assertFalse(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity3));
        assertNotEquals(null, frameBodyMagneticFluxDensity1);
        assertNotEquals(new Object(), frameBodyMagneticFluxDensity1);
    }

    @Test
    void testEqualsWithThreshold() throws IOException, InvalidSourceAndDestinationFrameTypeException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity2 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);
        final var frameBodyMagneticFluxDensity3 = new StandardDeviationFrameBodyMagneticFluxDensity();

        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity1, THRESHOLD));
        assertTrue(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity2, THRESHOLD));
        assertFalse(frameBodyMagneticFluxDensity1.equals(frameBodyMagneticFluxDensity3, THRESHOLD));
        assertFalse(frameBodyMagneticFluxDensity1.equals(null, THRESHOLD));
    }

    @Test
    void testClone() throws IOException, InvalidSourceAndDestinationFrameTypeException, CloneNotSupportedException {

        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);

        final var frameBodyMagneticFluxDensity2 = frameBodyMagneticFluxDensity1.clone();

        // check
        assertEquals(frameBodyMagneticFluxDensity1, frameBodyMagneticFluxDensity2);
    }

    @Test
    void testSerializeDeserialize() throws IOException, InvalidSourceAndDestinationFrameTypeException, 
            ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var magneticFluxDensityStandardDeviation = randomizer.nextDouble(0.0, MAX_MAGNETIC_FLUX_DENSITY);

        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        final var cbn = cnb.inverseAndReturnNew();

        final var bodyMagneticFluxDensity = createMagneticFluxDensity(position, timestamp, cnb);

        final var nedFrame = new NEDFrame(position, cbn);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var frameBodyMagneticFluxDensity1 = new StandardDeviationFrameBodyMagneticFluxDensity(
                bodyMagneticFluxDensity, ecefFrame, timestamp, magneticFluxDensityStandardDeviation);

        final var bytes = SerializationHelper.serialize(frameBodyMagneticFluxDensity1);
        final var frameBodyMagneticFluxDensity2 = SerializationHelper.deserialize(bytes);

        assertEquals(frameBodyMagneticFluxDensity1, frameBodyMagneticFluxDensity2);
        assertNotSame(frameBodyMagneticFluxDensity1, frameBodyMagneticFluxDensity2);
    }

    @Test
    void testSerialVersionUID() throws NoSuchFieldException, IllegalAccessException {
        final var field = StandardDeviationFrameBodyMagneticFluxDensity.class.getDeclaredField(
                "serialVersionUID");
        field.setAccessible(true);

        assertEquals(0L, field.get(null));
    }

    private static BodyMagneticFluxDensity createMagneticFluxDensity() throws IOException {
        final var position = createPosition();
        final var timestamp = createTimestamp();
        final var cnb = createAttitude();
        return createMagneticFluxDensity(position, timestamp, cnb);
    }

    private static BodyMagneticFluxDensity createMagneticFluxDensity(
            final NEDPosition position, final Date timestamp, final CoordinateTransformation cnb) throws IOException {
        final var wMMEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wMMEstimator.estimate(position, timestamp);
        return BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
    }

    private static CoordinateTransformation createAttitude() {
        final var randomizer = new UniformRandomizer();

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static NEDPosition createPosition() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static Date createTimestamp() {
        final var randomizer = new UniformRandomizer();
        return new Date(randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS));
    }
}
