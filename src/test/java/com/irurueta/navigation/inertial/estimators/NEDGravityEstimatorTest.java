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
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class NEDGravityEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double GRAVITY = 9.81;
    private static final double ABSOLUTE_ERROR = 0.03;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(NEDGravityEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0);
        assertEquals(NEDGravityEstimator.EARTH_POLAR_RADIUS_WGS84, Constants.EARTH_POLAR_RADIUS_WGS84, 0.0);
        assertEquals(NEDGravityEstimator.EARTH_ECCENTRICITY, Constants.EARTH_ECCENTRICITY, 0.0);
        assertEquals(NEDGravityEstimator.EARTH_FLATTENING_WGS84, Constants.EARTH_FLATTENING_WGS84, 0.0);
        assertEquals(NEDGravityEstimator.EARTH_GRAVITATIONAL_CONSTANT, Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(NEDGravityEstimator.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    void testEstimateWithCoordinates() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var estimator = new NEDGravityEstimator();
        final var gravity = new NEDGravity();
        estimator.estimate(latitude, height, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnWithCoordinates() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var estimator = new NEDGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(latitude, height);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithNEDFrame() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var frame = new NEDFrame(latitude, longitude, height);

        final var estimator = new NEDGravityEstimator();
        final var gravity = new NEDGravity();
        estimator.estimate(frame, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithNEDFrame() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var frame = new NEDFrame(latitude, longitude, height);

        final var estimator = new NEDGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(frame);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithAngleAndDistance() {
        final var randomizer = new UniformRandomizer();

        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var latitudeAngle = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var estimator = new NEDGravityEstimator();
        final var gravity = new NEDGravity();
        estimator.estimate(latitudeAngle, heightDistance, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithAngleAndDistance() {
        final var randomizer = new UniformRandomizer();

        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var latitudeAngle = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var estimator = new NEDGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(latitudeAngle, heightDistance);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithNEDPosition() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var position = new NEDPosition(latitude, longitude, height);

        final var estimator = new NEDGravityEstimator();
        final var gravity = new NEDGravity();
        estimator.estimate(position, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithNEDPosition() {
        final var randomizer = new UniformRandomizer();

        final var latitude = Math.toRadians(LATITUDE_DEGREES);
        final var longitude = Math.toRadians(LONGITUDE_DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var position = new NEDPosition(latitude, longitude, height);

        final var estimator = new NEDGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(position);

        final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                + Math.pow(gravity.getGe(), 2.0)
                + Math.pow(gravity.getGd(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateForMultipleLatitudes() {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final var gravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height);

            final var g = Math.sqrt(Math.pow(gravity.getGn(), 2.0)
                    + Math.pow(gravity.getGe(), 2.0)
                    + Math.pow(gravity.getGd(), 2.0));

            assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
        }
    }
}
