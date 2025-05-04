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

import static org.junit.jupiter.api.Assertions.*;

class NEDPositionEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-7;

    @Test
    void testEstimate() {

        final var estimator = new NEDPositionEstimator();

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result1 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, vn, ve, vd,
                result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, 
                result2);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final var result3 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result3);

        final var result4 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result4);

        final var result5 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, 
                oldVn, oldVe, oldVd, vn, ve, vd, result5);

        final var result6 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd,
                vn, ve, vd, result6);

        final var result7 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result7);

        final var result8 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result8);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var result9 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd, result9);

        final var result10 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd, result10);

        final var result11 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD,
                result11);

        final var result12 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, 
                result12);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var velocity = new NEDVelocity(vn, ve, vd);
        final var result13 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity,
                result13);

        final var result14 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity, result14);

        final var result15 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity,
                velocity, result15);

        final var result16 = new NEDPosition();
        estimator.estimate(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, velocity,
                result16);

        final var result17 = new NEDPosition();
        estimator.estimate(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity, result17);

        final var result18 = new NEDPosition();
        estimator.estimate(timeInterval, oldPosition, oldVelocity, velocity, result18);

        // check
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
    }

    @Test
    void testEstimateAndReturnNew() {
        final var estimator = new NEDPositionEstimator();

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result1 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = estimator.estimateAndReturnNew(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final var result3 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final var result4 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var result5 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final var result6 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final var result7 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var result8 = estimator.estimateAndReturnNew(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var result9 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd,
                vn, ve, vd);

        final var result10 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd);

        final var result11 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var result12 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var velocity = new NEDVelocity(vn, ve, vd);
        final var result13 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final var result14 = estimator.estimateAndReturnNew(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity);

        final var result15 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, velocity);

        final var result16 = estimator.estimateAndReturnNew(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity);

        final var result17 = estimator.estimateAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity);

        final var result18 = estimator.estimateAndReturnNew(timeInterval, oldPosition, oldVelocity, velocity);

        // check
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
    }

    @Test
    void testEstimatePosition() {

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result1 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result1);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                vn, ve, vd, result2);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final var result3 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result3);

        final var result4 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result4);

        final var result5 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd, result5);

        final var result6 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldVn, oldVe, oldVd, vn, ve, vd, result6);

        final var result7 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result7);

        final var result8 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD, result8);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var result9 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd,
                result9);

        final var result10 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldPosition, oldVn, oldVe, oldVd, vn, ve, vd, result10);

        final var result11 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result11);

        final var result12 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldPosition, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD, result12);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var velocity = new NEDVelocity(vn, ve, vd);
        final var result13 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result13);

        final var result14 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitude, oldLongitude, oldHeight,
                oldVelocity, velocity, result14);

        final var result15 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldVelocity, velocity, result15);

        final var result16 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldVelocity, velocity, result16);

        final var result17 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity, result17);

        final var result18 = new NEDPosition();
        NEDPositionEstimator.estimatePosition(timeInterval, oldPosition, oldVelocity, velocity, result18);

        // check
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
    }

    @Test
    void testEstimatePositionAndReturnNew() {

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result1 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, vn, ve, vd);

        final var timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final var result2 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, vn, ve, vd);

        final var oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final var oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final var oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final var oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final var oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final var speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final var speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final var speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final var result3 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final var result4 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldSpeedN, oldSpeedE, oldSpeedD,
                speedN, speedE, speedD);

        final var result5 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final var result6 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVn, oldVe, oldVd, vn, ve, vd);

        final var result7 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var result8 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);
        final var result9 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final var result10 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldVn, oldVe, oldVd, vn, ve, vd);

        final var result11 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldPosition, oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var result12 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval, oldPosition,
                oldSpeedN, oldSpeedE, oldSpeedD, speedN, speedE, speedD);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final var velocity = new NEDVelocity(vn, ve, vd);
        final var result13 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);

        final var result14 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitude, oldLongitude, oldHeight, oldVelocity, velocity);

        final var result15 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, velocity);

        final var result16 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance, oldVelocity, velocity);

        final var result17 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldPosition, oldVelocity, velocity);

        final var result18 = NEDPositionEstimator.estimatePositionAndReturnNew(timeInterval,
                oldPosition, oldVelocity, velocity);

        // check
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
    }

    @Test
    void testEstimateWithNegativeTimeIntervalThrowsIllegalArgumentException() {

        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result = new NEDPosition();
        assertThrows(IllegalArgumentException.class, () -> NEDPositionEstimator.estimatePosition(-TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd, vn, ve, vd, result));
    }

    @Test
    void testEstimateWithZeroTimeInterval() {
        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var result = new NEDPosition();
        NEDPositionEstimator.estimatePosition(0.0, oldLatitude, oldLongitude, oldHeight, oldVn, oldVe, oldVd,
                vn, ve, vd, result);

        assertEquals(oldLatitude, result.getLatitude(), ABSOLUTE_ERROR);
        assertEquals(oldLongitude, result.getLongitude(), ABSOLUTE_ERROR);
        assertEquals(oldHeight, result.getHeight(), ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithZeroVelocity() {
        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldVn = 0.0;
        final var oldVe = 0.0;
        final var oldVd = 0.0;

        final var vn = 0.0;
        final var ve = 0.0;
        final var vd = 0.0;

        final var result = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, oldHeight,
                oldVn, oldVe, oldVd, vn, ve, vd, result);

        assertEquals(oldLatitude, result.getLatitude(), ABSOLUTE_ERROR);
        assertEquals(oldLongitude, result.getLongitude(), ABSOLUTE_ERROR);
        assertEquals(oldHeight, result.getHeight(), ABSOLUTE_ERROR);
    }

    @Test
    void testCompareWithVelocityEstimator() {
        final var randomizer = new UniformRandomizer();
        final var oldLatitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldLongitude = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var oldHeight = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var oldPosition = new NEDPosition(oldLatitude, oldLongitude, oldHeight);

        final var oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);

        final var vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final var vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final var velocity = new NEDVelocity(vn, ve, vd);

        final var positionResult = new NEDPosition();
        NEDPositionEstimator.estimatePosition(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, velocity,
                positionResult);

        final var velocityResult = new NEDVelocity();
        NEDVelocityEstimator.estimateVelocity(TIME_INTERVAL_SECONDS, oldPosition, oldVelocity, positionResult,
                velocityResult);

        assertEquals(velocityResult.getVn(), vn, LARGE_ABSOLUTE_ERROR);
        assertEquals(velocityResult.getVe(), ve, LARGE_ABSOLUTE_ERROR);
        assertEquals(velocityResult.getVd(), vd, LARGE_ABSOLUTE_ERROR);
        assertTrue(velocityResult.equals(velocity, LARGE_ABSOLUTE_ERROR));

        final var positionResult2 = NEDPositionEstimator.estimatePositionAndReturnNew(TIME_INTERVAL_SECONDS,
                oldPosition, oldVelocity, velocityResult);

        assertTrue(positionResult2.equals(positionResult, ABSOLUTE_ERROR));
    }
}
