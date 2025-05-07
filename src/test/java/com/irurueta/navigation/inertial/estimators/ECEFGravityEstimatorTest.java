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

import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class ECEFGravityEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double GRAVITY = 9.81;
    private static final double ABSOLUTE_ERROR = 0.03;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(ECEFGravityEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 
                0.0);
        assertEquals(ECEFGravityEstimator.EARTH_GRAVITATIONAL_CONSTANT, Constants.EARTH_GRAVITATIONAL_CONSTANT, 
                0.0);
        assertEquals(ECEFGravityEstimator.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 
                Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(ECEFGravityEstimator.EARTH_ROTATION_RATE, Constants.EARTH_ROTATION_RATE, 0.0);
    }

    @Test
    void testEstimateWithCoordinates() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var estimator = new ECEFGravityEstimator();
        final var gravity = new ECEFGravity();
        estimator.estimate(x, y, z, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithCoordinates() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var estimator = new ECEFGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(x, y, z);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithECEFFrame() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = new ECEFGravity();
        estimator.estimate(ecefFrame, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithECEFFrame() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(ecefFrame);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithPosition() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var position = new InhomogeneousPoint3D(x, y, z);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = new ECEFGravity();
        estimator.estimate(position, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithPosition() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var position = new InhomogeneousPoint3D(x, y, z);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(position);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateWithDistances() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = new ECEFGravity();
        estimator.estimate(distanceX, distanceY, distanceZ, gravity);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateAndReturnNewWithDistances() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var x = ecefFrame.getX();
        final var y = ecefFrame.getY();
        final var z = ecefFrame.getZ();

        final var distanceX = new Distance(x, DistanceUnit.METER);
        final var distanceY = new Distance(y, DistanceUnit.METER);
        final var distanceZ = new Distance(z, DistanceUnit.METER);

        final var estimator = new ECEFGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(distanceX, distanceY, distanceZ);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateForAGivenLatitudeAndLongitude() {
        final var nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES), HEIGHT);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

        final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                + Math.pow(gravity.getGy(), 2.0)
                + Math.pow(gravity.getGz(), 2.0));

        assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
    }

    @Test
    void testEstimateForMultipleLatitudesAndLongitudes() {
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));

            final var nedFrame = new NEDFrame(latitude, longitude, HEIGHT);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            final var g = Math.sqrt(Math.pow(gravity.getGx(), 2.0)
                    + Math.pow(gravity.getGy(), 2.0)
                    + Math.pow(gravity.getGz(), 2.0));

            assertEquals(GRAVITY, g, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testEstimateAtCenterOfEarth() {
        final var estimator = new ECEFGravityEstimator();
        final var gravity = estimator.estimateAndReturnNew(0.0, 0.0, 0.0);

        // check
        assertEquals(0.0, gravity.getGx(), 0.0);
        assertEquals(0.0, gravity.getGy(), 0.0);
        assertEquals(0.0, gravity.getGz(), 0.0);
    }
}
