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
package com.irurueta.navigation.inertial.wmm;

import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class EarthMagneticFluxDensityEstimatorTest {

    private static final double MIN_MAGNITUDE = 30e-6;
    private static final double MAX_MAGNITUDE = 60e-6;

    private static final double MIN_DIP_DEGREES = -90.0;
    private static final double MAX_DIP_DEGREES = 90.0;

    private static final double MIN_DECLINATION = -180.0;
    private static final double MAX_DECLINATION = 180.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    void testEstimate() {
        final var randomizer = new UniformRandomizer();
        final var magnitude = randomizer.nextDouble(MIN_MAGNITUDE, MAX_MAGNITUDE);
        final var dip = Math.toRadians(randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final var declination = Math.toRadians(randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final var b1 = new NEDMagneticFluxDensity();
        EarthMagneticFluxDensityEstimator.estimate(magnitude, declination, dip, b1);
        final var b2 = EarthMagneticFluxDensityEstimator.estimate(magnitude, declination, dip);

        assertEquals(Math.cos(declination) * Math.cos(dip) * magnitude, b1.getBn(), 0.0);
        assertEquals(Math.sin(declination) * Math.cos(dip) * magnitude, b1.getBe(), 0.0);
        assertEquals(Math.sin(dip) * magnitude, b1.getBd(), 0.0);
        assertEquals(b1, b2);

        final var dipAngle = new Angle(dip, AngleUnit.RADIANS);
        final var declinationAngle = new Angle(declination, AngleUnit.RADIANS);

        final var b3 = new NEDMagneticFluxDensity();
        EarthMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle, b3);
        final var b4 = EarthMagneticFluxDensityEstimator.estimate(magnitude, declinationAngle, dipAngle);

        assertEquals(b1, b3);
        assertEquals(b1, b4);
    }

    @Test
    void testDeclination() {
        final var randomizer = new UniformRandomizer();
        final var magnitude = randomizer.nextDouble(MIN_MAGNITUDE, MAX_MAGNITUDE);
        final var dip = Math.toRadians(randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final var declination1 = Math.toRadians(randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final var b = EarthMagneticFluxDensityEstimator.estimate(magnitude, declination1, dip);

        // get declination
        final var declination2 = EarthMagneticFluxDensityEstimator.getDeclination(b);
        final var declination3 = EarthMagneticFluxDensityEstimator.getDeclinationAsAngle(b);
        final var declination4 = new Angle(0.0, AngleUnit.DEGREES);
        EarthMagneticFluxDensityEstimator.getDeclinationAsAngle(b, declination4);

        assertEquals(declination1, declination2, ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, declination3.getUnit());
        assertEquals(declination2, declination3.getValue().doubleValue(), 0.0);
        assertEquals(declination3, declination4);
    }

    @Test
    void testDip() {
        final var randomizer = new UniformRandomizer();
        final var magnitude = randomizer.nextDouble(MIN_MAGNITUDE, MAX_MAGNITUDE);
        final var dip1 = Math.toRadians(randomizer.nextDouble(MIN_DIP_DEGREES, MAX_DIP_DEGREES));
        final var declination = Math.toRadians(randomizer.nextDouble(MIN_DECLINATION, MAX_DECLINATION));

        final var b = EarthMagneticFluxDensityEstimator.estimate(magnitude, declination, dip1);

        // get dip
        final var dip2 = EarthMagneticFluxDensityEstimator.getDip(b);
        final var dip3 = EarthMagneticFluxDensityEstimator.getDipAsAngle(b);
        final var dip4 = new Angle(0.0, AngleUnit.DEGREES);
        EarthMagneticFluxDensityEstimator.getDipAsAngle(b, dip4);

        assertEquals(dip1, dip2, ABSOLUTE_ERROR);
        assertEquals(AngleUnit.RADIANS, dip3.getUnit());
        assertEquals(dip2, dip3.getValue().doubleValue(), 0.0);
        assertEquals(dip3, dip4);
    }
}
