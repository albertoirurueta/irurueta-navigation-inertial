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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.wmm.EarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;

import static org.junit.jupiter.api.Assertions.*;

class BodyMagneticFluxDensityEstimatorTest {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

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

    private static final double ABSOLUTE_ERROR = 1e-12;

    @Test
    void testDeclinationAndDip() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var declination1 = wmmEstimator.getDeclination(position, date);
        final var dip1 = wmmEstimator.getDip(position, date);

        final var b = wmmEstimator.estimate(position, date);

        // test that declination and dip values match for both WMM
        // and Earth magnetic flux density estimator.
        final var declination2 = EarthMagneticFluxDensityEstimator.getDeclination(b);
        final var dip2 = EarthMagneticFluxDensityEstimator.getDip(b);

        assertEquals(declination1, declination2, ABSOLUTE_ERROR);
        assertEquals(dip1, dip2, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimate1() throws IOException, WrongSizeException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var bEarth = wmmEstimator.estimate(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw, result1);

        final var b = bEarth.asMatrix();
        final var cnb = c.getMatrix();
        final var expected = cnb.multiplyAndReturnNew(b);

        assertArrayEquals(expected.getBuffer(), result1.asArray(), ABSOLUTE_ERROR);
    }

    @Test
    void testEstimate2() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw, result1);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate3() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c, result2);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate4() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c, result1);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate5() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw, result2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));
    }

    @Test
    void testEstimate6() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw, result1);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate7() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw, result1);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, c, result2);

        assertTrue(result1.equals(result2, ABSOLUTE_ERROR));
    }

    @Test
    void testEstimate8() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var c = createAttitude();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var result1 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, c, result1);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(bEarth, c);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate9() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle, 
                rollAngle, pitchAngle, yawAngle, result2);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate10() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, roll, pitch, yaw);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle, 
                rollAngle, pitchAngle, yawAngle);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate11() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();

        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var dipAngle = new Angle(dip, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle, c, result2);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate12() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var magnitude = wmmEstimator.getIntensity(position, date);
        final var declination = wmmEstimator.getDeclination(position, date);
        final var dip = wmmEstimator.getDip(position, date);

        final var c = createAttitude();

        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var dipAngle = new Angle(dip, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, c);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle, c);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate13() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw);
        final var result2 = new BodyMagneticFluxDensity();
        BodyMagneticFluxDensityEstimator.estimate(bEarth, rollAngle, pitchAngle, yawAngle, result2);

        assertEquals(result1, result2);
    }

    @Test
    void testEstimate14() throws IOException {
        final var position = createPosition();
        final var date = createTimestamp();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var bEarth = wmmEstimator.estimate(position, date);

        final var c = createAttitude();
        final var roll = c.getRollEulerAngle();
        final var pitch = c.getPitchEulerAngle();
        final var yaw = c.getYawEulerAngle();

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var yawAngle = new Angle(yaw, AngleUnit.RADIANS);

        final var result1 = BodyMagneticFluxDensityEstimator.estimate(bEarth, roll, pitch, yaw);
        final var result2 = BodyMagneticFluxDensityEstimator.estimate(bEarth, rollAngle, pitchAngle, yawAngle);

        assertEquals(result1, result2);
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
