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
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class ECIGravitationEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double GRAVITATION = 9.81;
    private static final double ABSOLUTE_ERROR = 0.03;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    @Test
    public void testConstants() {
        assertEquals(ECIGravitationEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                0.0);
        assertEquals(ECIGravitationEstimator.EARTH_GRAVITATIONAL_CONSTANT, Constants.EARTH_GRAVITATIONAL_CONSTANT, 0.0);
        assertEquals(ECIGravitationEstimator.EARTH_SECOND_GRAVITATIONAL_CONSTANT,
                Constants.EARTH_SECOND_GRAVITATIONAL_CONSTANT, 0.0);
    }

    @Test
    public void testEstimateWithCoordinates() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = new ECIGravitation();
        estimator.estimate(x, y, z, gravitation);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithCoordinates() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = estimator.estimateAndReturnNew(x, y, z);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithECIFrame() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = new ECIGravitation();
        estimator.estimate(eciFrame, gravitation);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithECIFrame() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = estimator.estimateAndReturnNew(eciFrame);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithPosition() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = new ECIGravitation();
        estimator.estimate(position, gravitation);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithPosition() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final Point3D position = new InhomogeneousPoint3D(x, y, z);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = estimator.estimateAndReturnNew(position);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateWithDistances() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = new ECIGravitation();
        estimator.estimate(distanceX, distanceY, distanceZ, gravitation);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAndReturnNewWithDistances() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);
        final double x = eciFrame.getX();
        final double y = eciFrame.getY();
        final double z = eciFrame.getZ();

        final Distance distanceX = new Distance(x, DistanceUnit.METER);
        final Distance distanceY = new Distance(y, DistanceUnit.METER);
        final Distance distanceZ = new Distance(z, DistanceUnit.METER);

        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = estimator.estimateAndReturnNew(distanceX, distanceY, distanceZ);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateForAGivenLatitudeAndLongitude() {
        final NEDFrame nedFrame = new NEDFrame(Math.toRadians(LATITUDE_DEGREES), Math.toRadians(LONGITUDE_DEGREES),
                HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);

        final ECIGravitation gravitation = ECIGravitationEstimator.estimateGravitationAndReturnNew(eciFrame);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateForMultipleLatitudesAndLongitudes() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));

        final NEDFrame nedFrame = new NEDFrame(latitude, longitude, HEIGHT);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final ECIFrame eciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame);

        final ECIGravitation gravitation = ECIGravitationEstimator.estimateGravitationAndReturnNew(eciFrame);

        final double g = Math.sqrt(Math.pow(gravitation.getGx(), 2.0) +
                Math.pow(gravitation.getGy(), 2.0) +
                Math.pow(gravitation.getGz(), 2.0));

        assertEquals(GRAVITATION, g, ABSOLUTE_ERROR);
    }

    @Test
    public void testEstimateAtCenterOfEarth() {
        final ECIGravitationEstimator estimator = new ECIGravitationEstimator();
        final ECIGravitation gravitation = estimator.estimateAndReturnNew(0.0, 0.0, 0.0);

        // check
        assertEquals(0.0, gravitation.getGx(), 0.0);
        assertEquals(0.0, gravitation.getGy(), 0.0);
        assertEquals(0.0, gravitation.getGz(), 0.0);
    }
}
