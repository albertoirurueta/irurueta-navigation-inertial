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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

class NEDVelocityEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    void testEstimate() {

        final var estimator = new NEDVelocityEstimator();

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var result1 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, latitude, longitude,
                height, result2);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var result3 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, longitudeAngle, heightDistance, result3);

        final var result4 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, longitudeAngle, heightDistance, result4);

        final var result5 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle, heightDistance, result5);

        final var result6 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle, heightDistance, result6);

        final var result7 = new NEDVelocity();
        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldVelocity, 
                latitude, longitude, height, result7);

        final var result8 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVelocity, latitude, longitude, height,
                result8);

        final var result9 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity,
                latitudeAngle, longitudeAngle, heightDistance, result9);

        final var result10 = new NEDVelocity();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity,
                latitudeAngle, longitudeAngle, heightDistance, result10);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd);
        final var result11 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude, height, result11);

        final var result12 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, latitude, longitude, height, result12);

        final var result13 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle, longitudeAngle, heightDistance, result13);

        final var result14 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, latitudeAngle, longitudeAngle, heightDistance, result14);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var position = new NEDPosition(latitude, longitude, height);
        final var result15 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd, position, result15);

        final var result16 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldVn, oldVe, oldVd, position, result16);

        final var result17 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, position, result17);

        final var result18 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, position, result18);

        final var result19 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, position, result19);

        final var result20 = new NEDVelocity();
        estimator.estimate(timeInterval, oldPosition, oldVelocity, position, result20);

        final var result21 = new NEDVelocity();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldFrame, position, result21);

        final var result22 = new NEDVelocity();
        estimator.estimate(timeInterval, oldFrame, position, result22);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    void testEstimateAndReturnNew() {

        final var estimator = new NEDVelocityEstimator();

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var result1 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = estimator.estimateAndReturnNew(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var result3 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result4 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, longitudeAngle, heightDistance);

        final var result5 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance);

        final var result6 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle, heightDistance);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result7 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                oldHeight, oldVelocity, latitude, longitude, height);

        final var result8 = estimator.estimateAndReturnNew(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, latitude, longitude, height);

        final var result9 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final var result10 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle, heightDistance);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd);
        final var result11 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                latitude, longitude, height);

        final var result12 = estimator.estimateAndReturnNew(timeInterval, oldFrame, latitude, longitude, height);

        final var result13 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result14 = estimator.estimateAndReturnNew(timeInterval, oldFrame,
                latitudeAngle, longitudeAngle, heightDistance);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var position = new NEDPosition(latitude, longitude, height);
        final var result15 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                position);

        final var result16 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldVn, oldVe, oldVd, position);

        final var result17 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, position);

        final var result18 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position);

        final var result19 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, position);

        final var result20 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldVelocity, position);

        final var result21 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame, position);

        final var result22 = estimator.estimateAndReturnNew(timeInterval, oldFrame, position);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    void testEstimateVelocity() {

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var result1 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, latitude, longitude, height, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                latitude, longitude, height, result2);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var result3 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, longitudeAngle, heightDistance,
                result3);

        final var result4 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, longitudeAngle, heightDistance,
                result4);

        final var result5 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle, heightDistance, result5);

        final var result6 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle, longitudeAngle, heightDistance, result6);

        final var result7 = new NEDVelocity();
        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height, result7);

        final var result8 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVelocity,
                latitude, longitude, height, result8);

        final var result9 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle, heightDistance, result9);

        final var result10 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle, heightDistance, result10);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd);
        final var result11 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldFrame, latitude, longitude, height, result11);

        final var result12 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldFrame, latitude, longitude, height, result12);

        final var result13 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldFrame, latitudeAngle, longitudeAngle,
                heightDistance, result13);

        final var result14 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldFrame, latitudeAngle, longitudeAngle, heightDistance,
                result14);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var position = new NEDPosition(latitude, longitude, height);
        final var result15 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd, position,
                result15);

        final var result16 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldPosition, oldVn, oldVe, oldVd, position, result16);

        final var result17 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                position, result17);

        final var result18 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, position,
                result18);

        final var result19 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, position, result19);

        final var result20 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldPosition, oldVelocity, position, result20);

        final var result21 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldFrame, position, result21);

        final var result22 = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(timeInterval, oldFrame, position, result22);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    void testEstimateVelocityAndReturnNew() {

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var result1 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, latitude, longitude, height);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, latitude, longitude, height);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var result3 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result4 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result5 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result6 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd, latitudeAngle,
                longitudeAngle, heightDistance);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var result7 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, latitude, longitude, height);

        final var result8 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, latitude, longitude, height);

        final var result9 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final var result10 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, latitudeAngle, longitudeAngle,
                heightDistance);

        final var oldFrame = new NEDFrame(oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd);
        final var result11 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                latitude, longitude, height);

        final var result12 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldFrame,
                latitude, longitude, height);

        final var result13 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                latitudeAngle, longitudeAngle, heightDistance);

        final var result14 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldFrame,
                latitudeAngle, longitudeAngle, heightDistance);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var position = new NEDPosition(latitude, longitude, height);
        final var result15 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                oldVn, oldVe, oldVd, position);

        final var result16 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, position);

        final var result17 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, position);

        final var result18 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, position);

        final var result19 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldPosition, oldVelocity, position);

        final var result20 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldPosition, oldVelocity,
                position);

        final var result21 = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS, oldFrame,
                position);

        final var result22 = NEDVelocityEstimator.estimateVelocityAndReturnNew(timeInterval, oldFrame, position);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
    }

    @Test
    void testEstimateWhenZeroTimeIntervalThrowsException() {
        final var estimator = new NEDVelocityEstimator();

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var result1 = new NEDVelocity();
        assertThrows(IllegalArgumentException.class, () -> estimator.estimate(0.0,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, latitude, longitude, height, result1));
    }

    @Test
    void testWhenNoPositionChange() {
        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result = NEDVelocityEstimator.estimateVelocityAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, oldLatitude, oldLongitude, oldHeight);

        assertEquals(result.getVn(), -oldVn, ABSOLUTE_ERROR);
        assertEquals(result.getVe(), -oldVe, ABSOLUTE_ERROR);
        assertEquals(result.getVd(), -oldVd, ABSOLUTE_ERROR);
    }
}
