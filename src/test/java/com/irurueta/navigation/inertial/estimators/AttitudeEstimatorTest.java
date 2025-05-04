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

import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMLoader;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import static org.junit.jupiter.api.Assertions.*;

class AttitudeEstimatorTest {

    private static final String RESOURCE = "wmm.cof";

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;
    private static final double LARGE_ABSOLUTE_ERROR = 4e-4;
    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    private static final int TIMES = 100;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    void testConstructor() throws IOException {
        final var estimator1 = new AttitudeEstimator();

        assertNotNull(estimator1);

        final var model = WMMLoader.loadFromResource(RESOURCE);
        final var estimator2 = new AttitudeEstimator(model);

        assertNotNull(estimator2);
    }

    @Test
    void testGetAttitude1() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, year);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        final var estimator = new AttitudeEstimator();
        estimator.getAttitude(latitude, longitude, height, year, fx, fy, fz, bx, by, bz, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude2() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, year);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var estimator = new AttitudeEstimator();
        final var result = estimator.getAttitude(latitude, longitude, height, year, fx, fy, fz, bx, by, bz);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude3() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, calendar);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        final var estimator = new AttitudeEstimator();
        estimator.getAttitude(latitude, longitude, height, calendar, fx, fy, fz, bx, by, bz, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude4() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, calendar);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var estimator = new AttitudeEstimator();
        final var result = estimator.getAttitude(latitude, longitude, height, calendar, fx, fy, fz, bx, by, bz);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, 8.0 * LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude5() throws IOException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var longitude = position.getLongitude();
            final var height = position.getHeight();

            final var timestamp = createTimestamp();
            final var date = new Date(timestamp);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
            final var fx = kinematics.getFx();
            final var fy = kinematics.getFy();
            final var fz = kinematics.getFz();

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, date);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
            final var bx = b.getBx();
            final var by = b.getBy();
            final var bz = b.getBz();

            final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

            final var estimator = new AttitudeEstimator();
            estimator.getAttitude(latitude, longitude, height, date, fx, fy, fz, bx, by, bz, result);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitude6() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var estimator = new AttitudeEstimator();
        final var result = estimator.getAttitude(latitude, longitude, height, date, fx, fy, fz, bx, by, bz);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude7() throws IOException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var height = position.getHeight();

            final var timestamp = createTimestamp();
            final var calendar = createCalendar(timestamp);
            final var year = createYear(calendar);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, year);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

            final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

            final var estimator = new AttitudeEstimator();
            estimator.getAttitude(position, year, kinematics, b, result);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitude8() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, year);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var estimator = new AttitudeEstimator();
        final var result = estimator.getAttitude(position, year, kinematics, b);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude9() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, calendar);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        final var estimator = new AttitudeEstimator();
        estimator.getAttitude(position, calendar, kinematics, b, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude10() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var calendar = createCalendar(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, calendar);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var estimator = new AttitudeEstimator();
        final var result = estimator.getAttitude(position, calendar, kinematics, b);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, VERY_LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude11() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);

        final var estimator = new AttitudeEstimator();
        estimator.getAttitude(position, date, kinematics, b, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitude12() throws IOException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var height = position.getHeight();

            final var timestamp = createTimestamp();
            final var date = new Date(timestamp);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, date);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

            final var estimator = new AttitudeEstimator();
            final var result = estimator.getAttitude(position, date, kinematics, b);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitudeStatic1() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic2() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic3() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic4() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = AttitudeEstimator.getAttitude(fx, fy, fz, bx, by, bz, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic5() throws IOException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var height = position.getHeight();

            final var timestamp = createTimestamp();
            final var date = new Date(timestamp);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, date);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

            final var declination = wmmEstimator.getDeclination(position, date);

            final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            AttitudeEstimator.getAttitude(kinematics, b, declination, result);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitudeStatic6() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = AttitudeEstimator.getAttitude(kinematics, b, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic7() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(kinematics, b, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic8() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = AttitudeEstimator.getAttitude(kinematics, b, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic9() throws IOException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {

            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var height = position.getHeight();

            final var timestamp = createTimestamp();
            final var date = new Date(timestamp);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
            final var fx = kinematics.getFx();
            final var fy = kinematics.getFy();
            final var fz = kinematics.getFz();

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, date);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
            final var bx = b.getBx();
            final var by = b.getBy();
            final var bz = b.getBz();

            final var declination = wmmEstimator.getDeclination(position, date);

            final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            AttitudeEstimator.getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination, result);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitudeStatic10() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getFx();
        final var fy = kinematics.getFy();
        final var fz = kinematics.getFz();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = AttitudeEstimator.getAttitude(latitude, height, fx, fy, fz, bx, by, bz, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic11() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
        final var fx = kinematics.getSpecificForceX();
        final var fy = kinematics.getSpecificForceY();
        final var fz = kinematics.getSpecificForceZ();

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(latitudeAngle, heightDistance, fx, fy, fz, bx, by, bz, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic12() throws IOException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {

            final var position = createPosition();
            final var latitude = position.getLatitude();
            final var height = position.getHeight();

            final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final var heightDistance = new Distance(height, DistanceUnit.METER);

            final var timestamp = createTimestamp();
            final var date = new Date(timestamp);

            // body attitude
            final var randomizer = new UniformRandomizer();
            final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // attitude is expressed as rotation from local navigation frame
            // to body frame, since angles are measured on the device body
            final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                    FrameType.BODY_FRAME);
            final var nedC = bodyC.inverseAndReturnNew();

            // get expected kinematics measure
            final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);
            final var fx = kinematics.getSpecificForceX();
            final var fy = kinematics.getSpecificForceY();
            final var fz = kinematics.getSpecificForceZ();

            final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
            final var earthB = wmmEstimator.estimate(position, date);
            final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
            final var bx = b.getBx();
            final var by = b.getBy();
            final var bz = b.getBz();

            final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

            final var result = AttitudeEstimator.getAttitude(latitudeAngle, heightDistance, fx, fy, fz, bx, by, bz,
                    declination);

            // check
            final var roll2 = result.getRollEulerAngle();
            final var pitch2 = result.getPitchEulerAngle();
            final var yaw2 = result.getYawEulerAngle();

            if (Math.abs(roll1 - roll2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(pitch1 - pitch2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(yaw1 - yaw2) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
            assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
            assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

            assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
            assertEquals(FrameType.BODY_FRAME, result.getDestinationType());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetAttitudeStatic13() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(position, kinematics, b, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic14() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclination(position, date);

        final var result = AttitudeEstimator.getAttitude(position, kinematics, b, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic15() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        AttitudeEstimator.getAttitude(position, kinematics, b, declination, result);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetAttitudeStatic16() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var height = position.getHeight();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final var nedC = bodyC.inverseAndReturnNew();

        // get expected kinematics measure
        final var kinematics = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL, nedC, nedC,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, latitude, height, latitude, height);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);

        final var declination = wmmEstimator.getDeclinationAsAngle(position, date);

        final var result = AttitudeEstimator.getAttitude(position, kinematics, b, declination);

        // check
        final var roll2 = result.getRollEulerAngle();
        final var pitch2 = result.getPitchEulerAngle();
        final var yaw2 = result.getYawEulerAngle();

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw1, yaw2, VERY_LARGE_ABSOLUTE_ERROR);

        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, result.getSourceType());
        assertEquals(FrameType.BODY_FRAME, result.getDestinationType());
    }

    @Test
    void testGetMagneticHeading() throws IOException {
        final var position = createPosition();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);
        final var magneticHeading1 = yaw - declination;

        final var magneticHeading2 = AttitudeEstimator.getMagneticHeading(bx, by, bz, roll, pitch);
        final var magneticHeading3 = AttitudeEstimator.getMagneticHeading(b, roll, pitch);

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var magneticHeading4 = AttitudeEstimator.getMagneticHeading(bx, by, bz, rollAngle, pitchAngle);
        final var magneticHeading5 = AttitudeEstimator.getMagneticHeading(b, rollAngle, pitchAngle);

        assertTrue(Math.abs(magneticHeading1 - magneticHeading2) <= ABSOLUTE_ERROR
                || Math.abs(2 * Math.PI + magneticHeading1 - magneticHeading2) <= ABSOLUTE_ERROR);
        assertEquals(magneticHeading2, magneticHeading3, 0.0);
        assertEquals(magneticHeading2, magneticHeading4, 0.0);
        assertEquals(magneticHeading2, magneticHeading5, 0.0);
    }

    @Test
    void testGetMagneticHeadingAsAngle() throws IOException {
        final var position = createPosition();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);
        final var magneticHeading1 = yaw - declination;

        final var magneticHeading2 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz, roll, pitch, magneticHeading2);
        final var magneticHeading3 = AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz, roll, pitch);
        final var magneticHeading4 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(b, roll, pitch, magneticHeading4);
        final var magneticHeading5 = AttitudeEstimator.getMagneticHeadingAsAngle(b, roll, pitch);

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);

        final var magneticHeading6 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz, rollAngle, pitchAngle, magneticHeading6);
        final var magneticHeading7 = AttitudeEstimator.getMagneticHeadingAsAngle(bx, by, bz, rollAngle, pitchAngle);
        final var magneticHeading8 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getMagneticHeadingAsAngle(b, rollAngle, pitchAngle, magneticHeading8);
        final var magneticHeading9 = AttitudeEstimator.getMagneticHeadingAsAngle(b, rollAngle, pitchAngle);

        assertTrue(Math.abs(magneticHeading1 - magneticHeading2.getValue().doubleValue()) <= ABSOLUTE_ERROR
                || Math.abs(2 * Math.PI + magneticHeading1 - magneticHeading2.getValue().doubleValue())
                <= ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, magneticHeading2.getUnit());
        assertEquals(magneticHeading2, magneticHeading3);
        assertEquals(magneticHeading2, magneticHeading4);
        assertEquals(magneticHeading2, magneticHeading5);
        assertEquals(magneticHeading2, magneticHeading6);
        assertEquals(magneticHeading2, magneticHeading7);
        assertEquals(magneticHeading2, magneticHeading8);
        assertEquals(magneticHeading2, magneticHeading9);
    }

    @Test
    void testGetYaw() throws IOException {
        final var position = createPosition();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);

        var yaw2 = AttitudeEstimator.getYaw(bx, by, bz, declination, roll, pitch);
        final var yaw3 = AttitudeEstimator.getYaw(b, declination, roll, pitch);

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var yaw4 = AttitudeEstimator.getYaw(bx, by, bz, declinationAngle, rollAngle, pitchAngle);
        final var yaw5 = AttitudeEstimator.getYaw(b, declinationAngle, rollAngle, pitchAngle);

        if (Math.abs(yaw2) > Math.PI) {
            yaw2 = 2.0 * Math.PI + yaw2;
        }
        assertEquals(yaw1, yaw2, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw4, ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw5, ABSOLUTE_ERROR);
    }

    @Test
    void testGetYawAsAngle() throws IOException {
        final var position = createPosition();

        final var timestamp = createTimestamp();
        final var date = new Date(timestamp);

        // body attitude
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final var bodyC = new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();
        final var earthB = wmmEstimator.estimate(position, date);
        final var b = BodyMagneticFluxDensityEstimator.estimate(earthB, bodyC);
        final var bx = b.getBx();
        final var by = b.getBy();
        final var bz = b.getBz();

        final var declination = wmmEstimator.getDeclination(position, date);

        final var yaw2 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(bx, by, bz, declination, roll, pitch, yaw2);
        final var yaw3 = AttitudeEstimator.getYawAsAngle(bx, by, bz, declination, roll, pitch);
        final var yaw4 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(b, declination, roll, pitch, yaw4);
        final var yaw5 = AttitudeEstimator.getYawAsAngle(b, declination, roll, pitch);

        final var rollAngle = new Angle(roll, AngleUnit.RADIANS);
        final var pitchAngle = new Angle(pitch, AngleUnit.RADIANS);
        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);
        final var yaw6 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(bx, by, bz, declinationAngle, rollAngle, pitchAngle, yaw6);
        final var yaw7 = AttitudeEstimator.getYawAsAngle(bx, by, bz, declinationAngle, rollAngle, pitchAngle);
        final var yaw8 = new Angle(0.0, AngleUnit.DEGREES);
        AttitudeEstimator.getYawAsAngle(b, declinationAngle, rollAngle, pitchAngle, yaw8);
        final var yaw9 = AttitudeEstimator.getYawAsAngle(b, declinationAngle, rollAngle, pitchAngle);

        assertEquals(yaw1, yaw2.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, yaw2.getUnit());
        assertEquals(yaw2, yaw3);
        assertEquals(yaw2, yaw4);
        assertEquals(yaw2, yaw5);
        assertEquals(yaw2, yaw6);
        assertEquals(yaw2, yaw7);
        assertEquals(yaw2, yaw8);
        assertEquals(yaw2, yaw9);
    }

    private static NEDPosition createPosition() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp() {
        final var randomizer = new UniformRandomizer();
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }

    private static GregorianCalendar createCalendar(final long timestamp) {
        final var calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        return calendar;
    }

    private static double createYear(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
    }
}
