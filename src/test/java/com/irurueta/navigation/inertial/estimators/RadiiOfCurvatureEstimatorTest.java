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

import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RadiiOfCurvatureEstimatorTest {
    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double EQUATOR_LATITUDE_DEGREES = 0.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testConstants() {
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                0.0);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_ECCENTRICITY, Constants.EARTH_ECCENTRICITY, 0.0);
    }

    @Test
    void testEstimate() {
        final var estimator = new RadiiOfCurvatureEstimator();

        final var radii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(LATITUDE_DEGREES), radii);

        final var northPoleRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(MAX_LATITUDE_DEGREES), northPoleRadii);

        final var southPoleRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(MIN_LATITUDE_DEGREES), southPoleRadii);

        final var equatorRadii = new RadiiOfCurvature();
        estimator.estimate(Math.toRadians(EQUATOR_LATITUDE_DEGREES), equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateAndReturnNew() {
        final var estimator = new RadiiOfCurvatureEstimator();

        final var radii = estimator.estimateAndReturnNew(Math.toRadians(LATITUDE_DEGREES));
        final var northPoleRadii = estimator.estimateAndReturnNew(Math.toRadians(MAX_LATITUDE_DEGREES));
        final var southPoleRadii = estimator.estimateAndReturnNew(Math.toRadians(MIN_LATITUDE_DEGREES));
        final var equatorRadii = estimator.estimateAndReturnNew(Math.toRadians(EQUATOR_LATITUDE_DEGREES));

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateRadiiOfCurvature() {
        final var radii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(Math.toRadians(LATITUDE_DEGREES), radii);

        final var northPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(Math.toRadians(MAX_LATITUDE_DEGREES), northPoleRadii);

        final var southPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(Math.toRadians(MIN_LATITUDE_DEGREES), southPoleRadii);

        final var equatorRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(Math.toRadians(EQUATOR_LATITUDE_DEGREES), equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateRadiiOfCurvatureAndAndReturnNew() {
        final var radii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(
                Math.toRadians(LATITUDE_DEGREES));
        final var northPoleRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(
                Math.toRadians(MAX_LATITUDE_DEGREES));
        final var southPoleRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(
                Math.toRadians(MIN_LATITUDE_DEGREES));
        final var equatorRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(
                Math.toRadians(EQUATOR_LATITUDE_DEGREES));

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateWithAngle() {
        final var estimator = new RadiiOfCurvatureEstimator();

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var radii = new RadiiOfCurvature();
        estimator.estimate(latitude, radii);

        final var maxLatitude = new Angle(MAX_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var northPoleRadii = new RadiiOfCurvature();
        estimator.estimate(maxLatitude, northPoleRadii);

        final var minLatitude = new Angle(MIN_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var southPoleRadii = new RadiiOfCurvature();
        estimator.estimate(minLatitude, southPoleRadii);

        final var equatorLatitude = new Angle(EQUATOR_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var equatorRadii = new RadiiOfCurvature();
        estimator.estimate(equatorLatitude, equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateAndReturnNewWithAngle() {
        final var estimator = new RadiiOfCurvatureEstimator();

        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var radii = estimator.estimateAndReturnNew(latitude);

        final var maxLatitude = new Angle(MAX_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var northPoleRadii = estimator.estimateAndReturnNew(maxLatitude);

        final var minLatitude = new Angle(MIN_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var southPoleRadii = estimator.estimateAndReturnNew(minLatitude);

        final var equatorLatitude = new Angle(EQUATOR_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var equatorRadii = estimator.estimateAndReturnNew(equatorLatitude);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateRadiiOfCurvatureWithAngle() {
        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var radii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(latitude, radii);

        final var maxLatitude = new Angle(MAX_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var northPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(maxLatitude, northPoleRadii);

        final var minLatitude = new Angle(MIN_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var southPoleRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(minLatitude, southPoleRadii);

        final var equatorLatitude = new Angle(EQUATOR_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var equatorRadii = new RadiiOfCurvature();
        RadiiOfCurvatureEstimator.estimateRadiiOfCurvature(equatorLatitude, equatorRadii);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }

    @Test
    void testEstimateRadiiOfCurvatureAndAndReturnNewWithAngle() {
        final var latitude = new Angle(LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var radii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(latitude);

        final var maxLatitude = new Angle(MAX_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var northPoleRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(maxLatitude);

        final var minLatitude = new Angle(MIN_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var southPoleRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(minLatitude);

        final var equatorLatitude = new Angle(EQUATOR_LATITUDE_DEGREES, AngleUnit.DEGREES);
        final var equatorRadii = RadiiOfCurvatureEstimator.estimateRadiiOfCurvatureAndReturnNew(equatorLatitude);

        assertTrue(radii.getRn() > equatorRadii.getRn());
        assertTrue(radii.getRn() < northPoleRadii.getRn());
        assertTrue(radii.getRn() < southPoleRadii.getRn());
        assertEquals(northPoleRadii.getRn(), southPoleRadii.getRn(), ABSOLUTE_ERROR);

        assertTrue(radii.getRe() > equatorRadii.getRe());
        assertTrue(radii.getRe() < northPoleRadii.getRe());
        assertTrue(radii.getRe() < southPoleRadii.getRe());
        assertEquals(northPoleRadii.getRe(), southPoleRadii.getRe(), ABSOLUTE_ERROR);
        assertEquals(RadiiOfCurvatureEstimator.EARTH_EQUATORIAL_RADIUS_WGS84, equatorRadii.getRe(), 0.0);
    }
}
