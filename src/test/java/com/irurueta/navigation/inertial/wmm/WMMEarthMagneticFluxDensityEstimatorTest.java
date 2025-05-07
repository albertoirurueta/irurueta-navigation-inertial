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

import com.irurueta.navigation.frames.NEDPosition;
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

class WMMEarthMagneticFluxDensityEstimatorTest {

    private static final String FILE_PATH = "./src/main/resources/com/irurueta/navigation/inertial/wmm/wmm.cof";

    private static final double TO_NANO = 1e9;

    private static final double ANGLE_ERROR = WMMEarthMagneticFluxDensityEstimator.ANGLE_ACCURACY_DEGREES;
    private static final double INTENSITY_ERROR = WMMEarthMagneticFluxDensityEstimator.INTENSITY_ACCURACY;
    private static final double TIME_ERROR = 0.0005;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2025, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2030, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    @Test
    void testConstants() {
        assertEquals(5e-3, WMMEarthMagneticFluxDensityEstimator.ANGLE_ACCURACY_DEGREES, 0.0);
        assertEquals(WMMEarthMagneticFluxDensityEstimator.ANGLE_ACCURACY_RADIANS,
                Math.toRadians(WMMEarthMagneticFluxDensityEstimator.ANGLE_ACCURACY_DEGREES), 0.0);
        assertEquals(5e-2, WMMEarthMagneticFluxDensityEstimator.INTENSITY_ACCURACY, 0.0);
    }

    @Test
    void testConstructor() throws IOException {
        final var estimator1 = new WMMEarthMagneticFluxDensityEstimator();
        assertNotNull(estimator1);
        assertNotNull(estimator1.getModel());

        final var model = WMMLoader.loadFromFile(FILE_PATH);
        final var estimator2 = new WMMEarthMagneticFluxDensityEstimator(model);
        assertNotNull(estimator2);
        assertSame(estimator2.getModel(), model);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new WMMEarthMagneticFluxDensityEstimator(null));
    }

    @Test
    void testDeclinationModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-112.46, Math.toDegrees(estimator.getDeclination(Math.toRadians(89.0),
                Math.toRadians(-121.0), 28e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-112.46, Math.toDegrees(estimator.getDeclination(Math.toRadians(89.0),
                Math.toRadians(-121.0), 28e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-112.44, Math.toDegrees(estimator.getDeclination(Math.toRadians(89.0),
                Math.toRadians(-121.0), 27e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-37.03, Math.toDegrees(estimator.getDeclination(Math.toRadians(80.0),
                Math.toRadians(-96), 48e3, 2020.0)), ANGLE_ERROR);
        assertEquals(50.78, Math.toDegrees(estimator.getDeclination(Math.toRadians(82.0),
                Math.toRadians(87.0), 54e3, 2020.0)), ANGLE_ERROR);
        assertEquals(0.60, Math.toDegrees(estimator.getDeclination(Math.toRadians(43.0),
                Math.toRadians(93.0), 65e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-5.84, Math.toDegrees(estimator.getDeclination(Math.toRadians(-33.0),
                Math.toRadians(109.0), 51e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-15.80, Math.toDegrees(estimator.getDeclination(Math.toRadians(-59.0),
                Math.toRadians(-8.0), 39e3, 2020.0)), ANGLE_ERROR);
        assertEquals(28.11, Math.toDegrees(estimator.getDeclination(Math.toRadians(-50.0),
                Math.toRadians(-103.0), 3e3, 2020.0)), ANGLE_ERROR);
        assertEquals(15.79, Math.toDegrees(estimator.getDeclination(Math.toRadians(-29.0),
                Math.toRadians(-110.0), 94e3, 2020.0)), ANGLE_ERROR);
        assertEquals(0.08, Math.toDegrees(estimator.getDeclination(Math.toRadians(14.0),
                Math.toRadians(143.0), 66e3, 2020.0)), ANGLE_ERROR);
        assertEquals(1.14, Math.toDegrees(estimator.getDeclination(0.0,
                Math.toRadians(21.0), 18e3, 2020.0)), ANGLE_ERROR);

        assertEquals(20.15, Math.toDegrees(estimator.getDeclination(Math.toRadians(-36.0),
                Math.toRadians(-137.0), 6e3, 2020.5)), ANGLE_ERROR);
        assertEquals(0.41, Math.toDegrees(estimator.getDeclination(Math.toRadians(26.0),
                Math.toRadians(81.0), 63e3, 2020.5)), ANGLE_ERROR);
        assertEquals(13.45, Math.toDegrees(estimator.getDeclination(Math.toRadians(38.0),
                Math.toRadians(-144.0), 69e3, 2020.5)), ANGLE_ERROR);
        assertEquals(57.39, Math.toDegrees(estimator.getDeclination(Math.toRadians(-70.0),
                Math.toRadians(-133.0), 50e3, 2020.5)), ANGLE_ERROR);
        assertEquals(15.40, Math.toDegrees(estimator.getDeclination(Math.toRadians(-52.0),
                Math.toRadians(-75.0), 8e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-32.58, Math.toDegrees(estimator.getDeclination(Math.toRadians(-66.0),
                Math.toRadians(17.0), 8e3, 2020.5)), ANGLE_ERROR);
        assertEquals(9.14, Math.toDegrees(estimator.getDeclination(Math.toRadians(-37.0),
                Math.toRadians(140.0), 22e3, 2020.5)), ANGLE_ERROR);
        assertEquals(10.80, Math.toDegrees(estimator.getDeclination(Math.toRadians(-12.0),
                Math.toRadians(-129.0), 40e3, 2020.5)), ANGLE_ERROR);
        assertEquals(11.47, Math.toDegrees(estimator.getDeclination(Math.toRadians(33.0),
                Math.toRadians(-118.0), 44e3, 2020.5)), ANGLE_ERROR);
        assertEquals(28.62, Math.toDegrees(estimator.getDeclination(Math.toRadians(-81.0),
                Math.toRadians(-67.0), 50e3, 2020.5)), ANGLE_ERROR);

        assertEquals(-22.32, Math.toDegrees(estimator.getDeclination(Math.toRadians(-57.0),
                Math.toRadians(3.0), 74e3, 2021.0)), ANGLE_ERROR);
        assertEquals(14.01, Math.toDegrees(estimator.getDeclination(Math.toRadians(-24.0),
                Math.toRadians(-122.0), 46e3, 2021.0)), ANGLE_ERROR);
        assertEquals(1.11, Math.toDegrees(estimator.getDeclination(Math.toRadians(23.0),
                Math.toRadians(63.0), 69e3, 2021.0)), ANGLE_ERROR);
        assertEquals(9.74, Math.toDegrees(estimator.getDeclination(Math.toRadians(-3.0),
                Math.toRadians(-147.0), 33e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-6.08, Math.toDegrees(estimator.getDeclination(Math.toRadians(-72.0),
                Math.toRadians(-22.0), 47e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-1.74, Math.toDegrees(estimator.getDeclination(Math.toRadians(-14.0),
                Math.toRadians(99.0), 62e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-36.73, Math.toDegrees(estimator.getDeclination(Math.toRadians(86.0),
                Math.toRadians(-46.0), 83e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-80.81, Math.toDegrees(estimator.getDeclination(Math.toRadians(-64.0),
                Math.toRadians(87.0), 82e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-14.29, Math.toDegrees(estimator.getDeclination(Math.toRadians(-19.0),
                Math.toRadians(43.0), 34e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-59.04, Math.toDegrees(estimator.getDeclination(Math.toRadians(-81.0),
                Math.toRadians(40.0), 56e3, 2021.0)), ANGLE_ERROR);

        assertEquals(-3.38, Math.toDegrees(estimator.getDeclination(0.0,
                Math.toRadians(80.0), 14e3, 2021.5)), ANGLE_ERROR);
        assertEquals(30.31, Math.toDegrees(estimator.getDeclination(Math.toRadians(-82.0),
                Math.toRadians(-68.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-11.55, Math.toDegrees(estimator.getDeclination(Math.toRadians(-46.0),
                Math.toRadians(-42.0), 44e3, 2021.5)), ANGLE_ERROR);
        assertEquals(1.24, Math.toDegrees(estimator.getDeclination(Math.toRadians(17.0),
                Math.toRadians(52.0), 43e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-1.68, Math.toDegrees(estimator.getDeclination(Math.toRadians(10.0),
                Math.toRadians(78.0), 64e3, 2021.5)), ANGLE_ERROR);
        assertEquals(12.39, Math.toDegrees(estimator.getDeclination(Math.toRadians(33.0),
                Math.toRadians(-145.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-136.36, Math.toDegrees(estimator.getDeclination(Math.toRadians(-79.0),
                Math.toRadians(115.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(18.12, Math.toDegrees(estimator.getDeclination(Math.toRadians(-33.0),
                Math.toRadians(-114), 14e3, 2021.5)), ANGLE_ERROR);
        assertEquals(2.15, Math.toDegrees(estimator.getDeclination(Math.toRadians(29.0),
                Math.toRadians(66.0), 19e3, 2021.5)), ANGLE_ERROR);
        assertEquals(10.17, Math.toDegrees(estimator.getDeclination(Math.toRadians(-11.0),
                Math.toRadians(167.0), 86e3, 2021.5)), ANGLE_ERROR);

        assertEquals(-17.00, Math.toDegrees(estimator.getDeclination(Math.toRadians(-66.0),
                Math.toRadians(-5.0), 37e3, 2022.0)), ANGLE_ERROR);
        assertEquals(15.42, Math.toDegrees(estimator.getDeclination(Math.toRadians(72.0),
                Math.toRadians(-115.0), 67e3, 2022.0)), ANGLE_ERROR);
        assertEquals(6.59, Math.toDegrees(estimator.getDeclination(Math.toRadians(22.0),
                Math.toRadians(174.0), 44e3, 2022.0)), ANGLE_ERROR);
        assertEquals(1.48, Math.toDegrees(estimator.getDeclination(Math.toRadians(54.0),
                Math.toRadians(178.0), 54e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-47.44, Math.toDegrees(estimator.getDeclination(Math.toRadians(-43.0),
                Math.toRadians(50.0), 57e3, 2022.0)), ANGLE_ERROR);
        assertEquals(24.34, Math.toDegrees(estimator.getDeclination(Math.toRadians(-43.0),
                Math.toRadians(-111.0), 44e3, 2022.0)), ANGLE_ERROR);
        assertEquals(57.10, Math.toDegrees(estimator.getDeclination(Math.toRadians(-63.0),
                Math.toRadians(178.0), 12e3, 2022.0)), ANGLE_ERROR);
        assertEquals(8.78, Math.toDegrees(estimator.getDeclination(Math.toRadians(27.0),
                Math.toRadians(-169.0), 38e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-17.66, Math.toDegrees(estimator.getDeclination(Math.toRadians(59.0),
                Math.toRadians(-77.0), 61e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-14.09, Math.toDegrees(estimator.getDeclination(Math.toRadians(-47.0),
                Math.toRadians(-32.0), 67e3, 2022.0)), ANGLE_ERROR);

        assertEquals(18.90, Math.toDegrees(estimator.getDeclination(Math.toRadians(62.0),
                Math.toRadians(53.0), 8e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-15.95, Math.toDegrees(estimator.getDeclination(Math.toRadians(-68.0),
                Math.toRadians(-7.0), 77e3, 2022.5)), ANGLE_ERROR);
        assertEquals(7.82, Math.toDegrees(estimator.getDeclination(Math.toRadians(-5.0),
                Math.toRadians(159.0), 98e3, 2022.5)), ANGLE_ERROR);
        assertEquals(15.73, Math.toDegrees(estimator.getDeclination(Math.toRadians(-29.0),
                Math.toRadians(-107.0), 34e3, 2022.5)), ANGLE_ERROR);
        assertEquals(1.77, Math.toDegrees(estimator.getDeclination(Math.toRadians(27.0),
                Math.toRadians(65.0), 60e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-101.47, Math.toDegrees(estimator.getDeclination(Math.toRadians(-72.0),
                Math.toRadians(95.0), 73e3, 2022.5)), ANGLE_ERROR);
        assertEquals(18.38, Math.toDegrees(estimator.getDeclination(Math.toRadians(-46.0),
                Math.toRadians(-85.0), 96e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-16.63, Math.toDegrees(estimator.getDeclination(Math.toRadians(-13.0),
                Math.toRadians(-59.0), 0e3, 2022.5)), ANGLE_ERROR);
        assertEquals(2.01, Math.toDegrees(estimator.getDeclination(Math.toRadians(66.0),
                Math.toRadians(-178), 16e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-64.70, Math.toDegrees(estimator.getDeclination(Math.toRadians(-87.0),
                Math.toRadians(38.0), 72e3, 2022.5)), ANGLE_ERROR);

        assertEquals(5.20, Math.toDegrees(estimator.getDeclination(Math.toRadians(20.0),
                Math.toRadians(167.0), 49e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-7.30, Math.toDegrees(estimator.getDeclination(Math.toRadians(5.0),
                Math.toRadians(-13.0), 71e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-0.57, Math.toDegrees(estimator.getDeclination(Math.toRadians(14.0),
                Math.toRadians(65.0), 95e3, 2023.0)), ANGLE_ERROR);
        assertEquals(41.70, Math.toDegrees(estimator.getDeclination(Math.toRadians(-85.0),
                Math.toRadians(-79.0), 86e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-3.86, Math.toDegrees(estimator.getDeclination(Math.toRadians(-36.0),
                Math.toRadians(-64.0), 30e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-14.23, Math.toDegrees(estimator.getDeclination(Math.toRadians(79.0),
                Math.toRadians(125.0), 75e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-15.22, Math.toDegrees(estimator.getDeclination(Math.toRadians(6.0),
                Math.toRadians(-32.0), 21e3, 2023.0)), ANGLE_ERROR);
        assertEquals(30.30, Math.toDegrees(estimator.getDeclination(Math.toRadians(-76.0),
                Math.toRadians(-75.0), 1e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-11.91, Math.toDegrees(estimator.getDeclination(Math.toRadians(-46),
                Math.toRadians(-41.0), 45e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-24.09, Math.toDegrees(estimator.getDeclination(Math.toRadians(-22.0),
                Math.toRadians(-21.0), 11e3, 2023.0)), ANGLE_ERROR);

        assertEquals(16.17, Math.toDegrees(estimator.getDeclination(Math.toRadians(54.0),
                Math.toRadians(-120.0), 28e3, 2023.5)), ANGLE_ERROR);
        assertEquals(40.47, Math.toDegrees(estimator.getDeclination(Math.toRadians(-58.0),
                Math.toRadians(156.0), 68e3, 2023.5)), ANGLE_ERROR);
        assertEquals(29.82, Math.toDegrees(estimator.getDeclination(Math.toRadians(-65.0),
                Math.toRadians(-88.0), 39e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-13.92, Math.toDegrees(estimator.getDeclination(Math.toRadians(-23.0),
                Math.toRadians(81.0), 27e3, 2023.5)), ANGLE_ERROR);
        assertEquals(1.01, Math.toDegrees(estimator.getDeclination(Math.toRadians(34.0),
                0.0, 11e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-66.99, Math.toDegrees(estimator.getDeclination(Math.toRadians(-62.0),
                Math.toRadians(65.0), 72e3, 2023.5)), ANGLE_ERROR);
        assertEquals(61.14, Math.toDegrees(estimator.getDeclination(Math.toRadians(86.0),
                Math.toRadians(70.0), 55e3, 2023.5)), ANGLE_ERROR);
        assertEquals(0.37, Math.toDegrees(estimator.getDeclination(Math.toRadians(32.0),
                Math.toRadians(163.0), 59e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-9.35, Math.toDegrees(estimator.getDeclination(Math.toRadians(48.0),
                Math.toRadians(148.0), 65e3, 2023.5)), ANGLE_ERROR);
        assertEquals(4.39, Math.toDegrees(estimator.getDeclination(Math.toRadians(30.0),
                Math.toRadians(28.0), 95e3, 2023.5)), ANGLE_ERROR);

        assertEquals(8.84, Math.toDegrees(estimator.getDeclination(Math.toRadians(-60.0),
                Math.toRadians(-59.0), 95e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-54.27, Math.toDegrees(estimator.getDeclination(Math.toRadians(-70.0),
                Math.toRadians(42.0), 95e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-85.27, Math.toDegrees(estimator.getDeclination(Math.toRadians(87.0),
                Math.toRadians(-154.0), 50e3, 2024.0)), ANGLE_ERROR);
        assertEquals(3.82, Math.toDegrees(estimator.getDeclination(Math.toRadians(32.0),
                Math.toRadians(19.0), 58e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-2.67, Math.toDegrees(estimator.getDeclination(Math.toRadians(34.0),
                Math.toRadians(-13.0), 57e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-63.48, Math.toDegrees(estimator.getDeclination(Math.toRadians(-76.0),
                Math.toRadians(49.0), 38e3, 2024.0)), ANGLE_ERROR);
        assertEquals(31.55, Math.toDegrees(estimator.getDeclination(Math.toRadians(-50.0),
                Math.toRadians(-179.0), 49e3, 2024.0)), ANGLE_ERROR);
        assertEquals(38.06, Math.toDegrees(estimator.getDeclination(Math.toRadians(-55.0),
                Math.toRadians(-171.0), 90e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-5.03, Math.toDegrees(estimator.getDeclination(Math.toRadians(42.0),
                Math.toRadians(-19.0), 41e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-6.64, Math.toDegrees(estimator.getDeclination(Math.toRadians(46.0),
                Math.toRadians(-22.0), 19e3, 2024.0)), ANGLE_ERROR);

        assertEquals(9.23, Math.toDegrees(estimator.getDeclination(Math.toRadians(13.0),
                Math.toRadians(-132.0), 31e3, 2024.5)), ANGLE_ERROR);
        assertEquals(7.14, Math.toDegrees(estimator.getDeclination(Math.toRadians(-2.0),
                Math.toRadians(158.0), 93e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-55.60, Math.toDegrees(estimator.getDeclination(Math.toRadians(-76.0),
                Math.toRadians(40.0), 51e3, 2024.5)), ANGLE_ERROR);
        assertEquals(10.51, Math.toDegrees(estimator.getDeclination(Math.toRadians(22.0),
                Math.toRadians(-132.0), 64e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-62.58, Math.toDegrees(estimator.getDeclination(Math.toRadians(-65.0),
                Math.toRadians(55.0), 26e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-13.62, Math.toDegrees(estimator.getDeclination(Math.toRadians(-21.0),
                Math.toRadians(32.0), 66e3, 2024.5)), ANGLE_ERROR);
        assertEquals(9.24, Math.toDegrees(estimator.getDeclination(Math.toRadians(9.0),
                Math.toRadians(-172.0), 18e3, 2024.5)), ANGLE_ERROR);
        assertEquals(29.69, Math.toDegrees(estimator.getDeclination(Math.toRadians(88.0),
                Math.toRadians(26.0), 63e3, 2024.5)), ANGLE_ERROR);
        assertEquals(0.50, Math.toDegrees(estimator.getDeclination(Math.toRadians(17.0),
                Math.toRadians(5.0), 33e3, 2024.5)), ANGLE_ERROR);
        assertEquals(4.64, Math.toDegrees(estimator.getDeclination(Math.toRadians(-18.0),
                Math.toRadians(138.0), 77e3, 2024.5)), ANGLE_ERROR);
    }

    @Test
    void testDeclinationModel2025() throws IOException {
        // test values correspond to D (Deg) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(1.28, Math.toDegrees(estimator.getDeclination(Math.toRadians(80.0),
                Math.toRadians(0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(-0.16, Math.toDegrees(estimator.getDeclination(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(68.78, Math.toDegrees(estimator.getDeclination(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(0.85, Math.toDegrees(estimator.getDeclination(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2025.0)), ANGLE_ERROR);
        assertEquals(-0.15, Math.toDegrees(estimator.getDeclination(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2025.0)), ANGLE_ERROR);
        assertEquals(68.21, Math.toDegrees(estimator.getDeclination(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2025.0)), ANGLE_ERROR);

        assertEquals(2.59, Math.toDegrees(estimator.getDeclination(Math.toRadians(80.0),
                Math.toRadians(0.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(-0.24, Math.toDegrees(estimator.getDeclination(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(68.49, Math.toDegrees(estimator.getDeclination(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(2.16, Math.toDegrees(estimator.getDeclination(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2027.5)), ANGLE_ERROR);
        assertEquals(-0.23, Math.toDegrees(estimator.getDeclination(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2027.5)), ANGLE_ERROR);
        assertEquals(67.93, Math.toDegrees(estimator.getDeclination(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2027.5)), ANGLE_ERROR);
    }

    @Test
    void testDipModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(88.48, Math.toDegrees(estimator.getDip(Math.toRadians(89.0),
                Math.toRadians(-121.0), 28e3, 2020.0)), ANGLE_ERROR);
        assertEquals(88.04, Math.toDegrees(estimator.getDip(Math.toRadians(80.0),
                Math.toRadians(-96.0), 48e3, 2020.0)), ANGLE_ERROR);
        assertEquals(87.49, Math.toDegrees(estimator.getDip(Math.toRadians(82.0),
                Math.toRadians(87.0), 54e3, 2020.0)), ANGLE_ERROR);
        assertEquals(63.94, Math.toDegrees(estimator.getDip(Math.toRadians(43.0),
                Math.toRadians(93.0), 65e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-67.63, Math.toDegrees(estimator.getDip(Math.toRadians(-33.0),
                Math.toRadians(109.0), 51e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-58.82, Math.toDegrees(estimator.getDip(Math.toRadians(-59.0),
                Math.toRadians(-8.0), 39e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-55.01, Math.toDegrees(estimator.getDip(Math.toRadians(-50.0),
                Math.toRadians(-103.0), 3e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-38.38, Math.toDegrees(estimator.getDip(Math.toRadians(-29.0),
                Math.toRadians(-110.0), 94e3, 2020.0)), ANGLE_ERROR);
        assertEquals(12.84, Math.toDegrees(estimator.getDip(Math.toRadians(14.0),
                Math.toRadians(143.0), 66e3, 2020.0)), ANGLE_ERROR);
        assertEquals(-26.45, Math.toDegrees(estimator.getDip(0.0,
                Math.toRadians(21.0), 18e3, 2020.0)), ANGLE_ERROR);

        assertEquals(-52.18, Math.toDegrees(estimator.getDip(Math.toRadians(-36.0),
                Math.toRadians(-137), 6e3, 2020.5)), ANGLE_ERROR);
        assertEquals(40.96, Math.toDegrees(estimator.getDip(Math.toRadians(26.0),
                Math.toRadians(81.0), 63e3, 2020.5)), ANGLE_ERROR);
        assertEquals(57.01, Math.toDegrees(estimator.getDip(Math.toRadians(38.0),
                Math.toRadians(-144.0), 69e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-72.17, Math.toDegrees(estimator.getDip(Math.toRadians(-70.0),
                Math.toRadians(-133.0), 50e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-49.49, Math.toDegrees(estimator.getDip(Math.toRadians(-52.0),
                Math.toRadians(-75.0), 8e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-59.76, Math.toDegrees(estimator.getDip(Math.toRadians(-66.0),
                Math.toRadians(17.0), 8e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-68.64, Math.toDegrees(estimator.getDip(Math.toRadians(-37.0),
                Math.toRadians(140.0), 22e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-15.62, Math.toDegrees(estimator.getDip(Math.toRadians(-12.0),
                Math.toRadians(-129.0), 40e3, 2020.5)), ANGLE_ERROR);
        assertEquals(57.99, Math.toDegrees(estimator.getDip(Math.toRadians(33.0),
                Math.toRadians(-118.0), 44e3, 2020.5)), ANGLE_ERROR);
        assertEquals(-67.71, Math.toDegrees(estimator.getDip(Math.toRadians(-81.0),
                Math.toRadians(-67.0), 50e3, 2020.5)), ANGLE_ERROR);

        assertEquals(-59.06, Math.toDegrees(estimator.getDip(Math.toRadians(-57.0),
                Math.toRadians(3.0), 74e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-34.26, Math.toDegrees(estimator.getDip(Math.toRadians(-24.0),
                Math.toRadians(-122.0), 46e3, 2021.0)), ANGLE_ERROR);
        assertEquals(35.87, Math.toDegrees(estimator.getDip(Math.toRadians(23.0),
                Math.toRadians(63.0), 69e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-2.31, Math.toDegrees(estimator.getDip(Math.toRadians(-3.0),
                Math.toRadians(-147.0), 33e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-61.24, Math.toDegrees(estimator.getDip(Math.toRadians(-72.0),
                Math.toRadians(-22.0), 47e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-45.06, Math.toDegrees(estimator.getDip(Math.toRadians(-14.0),
                Math.toRadians(99.0), 62e3, 2021.0)), ANGLE_ERROR);
        assertEquals(86.83, Math.toDegrees(estimator.getDip(Math.toRadians(86.0),
                Math.toRadians(-46.0), 83e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-75.25, Math.toDegrees(estimator.getDip(Math.toRadians(-64.0),
                Math.toRadians(87.0), 82e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-52.45, Math.toDegrees(estimator.getDip(Math.toRadians(-19.0),
                Math.toRadians(43.0), 34e3, 2021.0)), ANGLE_ERROR);
        assertEquals(-68.53, Math.toDegrees(estimator.getDip(Math.toRadians(-81.0),
                Math.toRadians(40.0), 56e3, 2021.0)), ANGLE_ERROR);

        assertEquals(-17.26, Math.toDegrees(estimator.getDip(0.0,
                Math.toRadians(80.0), 14e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-68.17, Math.toDegrees(estimator.getDip(Math.toRadians(-82.0),
                Math.toRadians(-68.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-53.80, Math.toDegrees(estimator.getDip(Math.toRadians(-46.0),
                Math.toRadians(-42.0), 44e3, 2021.5)), ANGLE_ERROR);
        assertEquals(23.87, Math.toDegrees(estimator.getDip(Math.toRadians(17.0),
                Math.toRadians(52.0), 43e3, 2021.5)), ANGLE_ERROR);
        assertEquals(7.43, Math.toDegrees(estimator.getDip(Math.toRadians(10.0),
                Math.toRadians(78.0), 64e3, 2021.5)), ANGLE_ERROR);
        assertEquals(52.51, Math.toDegrees(estimator.getDip(Math.toRadians(33.0),
                Math.toRadians(-145.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-77.44, Math.toDegrees(estimator.getDip(Math.toRadians(-79.0),
                Math.toRadians(115.0), 12e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-44.22, Math.toDegrees(estimator.getDip(Math.toRadians(-33.0),
                Math.toRadians(-114.0), 14e3, 2021.5)), ANGLE_ERROR);
        assertEquals(45.98, Math.toDegrees(estimator.getDip(Math.toRadians(29.0),
                Math.toRadians(66.0), 19e3, 2021.5)), ANGLE_ERROR);
        assertEquals(-31.47, Math.toDegrees(estimator.getDip(Math.toRadians(-11.0),
                Math.toRadians(167.0), 86e3, 2021.5)), ANGLE_ERROR);

        assertEquals(-59.24, Math.toDegrees(estimator.getDip(Math.toRadians(-66.0),
                Math.toRadians(-5.0), 37e3, 2022.0)), ANGLE_ERROR);
        assertEquals(85.20, Math.toDegrees(estimator.getDip(Math.toRadians(72.0),
                Math.toRadians(-115.0), 67e3, 2022.0)), ANGLE_ERROR);
        assertEquals(31.89, Math.toDegrees(estimator.getDip(Math.toRadians(22.0),
                Math.toRadians(174.0), 44e3, 2022.0)), ANGLE_ERROR);
        assertEquals(65.39, Math.toDegrees(estimator.getDip(Math.toRadians(54.0),
                Math.toRadians(178.0), 54e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-62.96, Math.toDegrees(estimator.getDip(Math.toRadians(-43.0),
                Math.toRadians(50.0), 57e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-52.69, Math.toDegrees(estimator.getDip(Math.toRadians(-43.0),
                Math.toRadians(-111.0), 44e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-79.32, Math.toDegrees(estimator.getDip(Math.toRadians(-63.0),
                Math.toRadians(178.0), 12e3, 2022.0)), ANGLE_ERROR);
        assertEquals(42.60, Math.toDegrees(estimator.getDip(Math.toRadians(27.0),
                Math.toRadians(-169.0), 38e3, 2022.0)), ANGLE_ERROR);
        assertEquals(79.05, Math.toDegrees(estimator.getDip(Math.toRadians(59.0),
                Math.toRadians(-77.0), 61e3, 2022.0)), ANGLE_ERROR);
        assertEquals(-57.59, Math.toDegrees(estimator.getDip(Math.toRadians(-47.0),
                Math.toRadians(-32.0), 67e3, 2022.0)), ANGLE_ERROR);

        assertEquals(76.52, Math.toDegrees(estimator.getDip(Math.toRadians(62.0),
                Math.toRadians(53.0), 8e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-59.98, Math.toDegrees(estimator.getDip(Math.toRadians(-68.0),
                Math.toRadians(-7.0), 77e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-23.04, Math.toDegrees(estimator.getDip(Math.toRadians(-5.0),
                Math.toRadians(159.0), 98e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-37.60, Math.toDegrees(estimator.getDip(Math.toRadians(-29.0),
                Math.toRadians(-107.0), 34e3, 2022.5)), ANGLE_ERROR);
        assertEquals(42.78, Math.toDegrees(estimator.getDip(Math.toRadians(27.0),
                Math.toRadians(65.0), 60e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-76.46, Math.toDegrees(estimator.getDip(Math.toRadians(-72.0),
                Math.toRadians(95.0), 73e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-47.34, Math.toDegrees(estimator.getDip(Math.toRadians(-46.0),
                Math.toRadians(-85.0), 96e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-13.34, Math.toDegrees(estimator.getDip(Math.toRadians(-13.0),
                Math.toRadians(-59.0), 0e3, 2022.5)), ANGLE_ERROR);
        assertEquals(75.65, Math.toDegrees(estimator.getDip(Math.toRadians(66.0),
                Math.toRadians(-178.0), 16e3, 2022.5)), ANGLE_ERROR);
        assertEquals(-71.07, Math.toDegrees(estimator.getDip(Math.toRadians(-87.0),
                Math.toRadians(38.0), 72e3, 2022.5)), ANGLE_ERROR);

        assertEquals(26.88, Math.toDegrees(estimator.getDip(Math.toRadians(20.0),
                Math.toRadians(167.0), 49e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-17.29, Math.toDegrees(estimator.getDip(Math.toRadians(5.0),
                Math.toRadians(-13.0), 71e3, 2023.0)), ANGLE_ERROR);
        assertEquals(17.38, Math.toDegrees(estimator.getDip(Math.toRadians(14.0),
                Math.toRadians(65.0), 95e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-70.37, Math.toDegrees(estimator.getDip(Math.toRadians(-85.0),
                Math.toRadians(-79.0), 86e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-39.32, Math.toDegrees(estimator.getDip(Math.toRadians(-36.0),
                Math.toRadians(-64.0), 30e3, 2023.0)), ANGLE_ERROR);
        assertEquals(87.30, Math.toDegrees(estimator.getDip(Math.toRadians(79.0),
                Math.toRadians(125.0), 75e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-7.14, Math.toDegrees(estimator.getDip(Math.toRadians(6.0),
                Math.toRadians(-32.0), 21e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-65.31, Math.toDegrees(estimator.getDip(Math.toRadians(-76.0),
                Math.toRadians(-75.0), 1e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-54.38, Math.toDegrees(estimator.getDip(Math.toRadians(-46.0),
                Math.toRadians(-41.0), 45e3, 2023.0)), ANGLE_ERROR);
        assertEquals(-56.69, Math.toDegrees(estimator.getDip(Math.toRadians(-22.0),
                Math.toRadians(-21.0), 11e3, 2023.0)), ANGLE_ERROR);

        assertEquals(74.00, Math.toDegrees(estimator.getDip(Math.toRadians(54.0),
                Math.toRadians(-120.0), 28e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-81.59, Math.toDegrees(estimator.getDip(Math.toRadians(-58.0),
                Math.toRadians(156.0), 68e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-60.27, Math.toDegrees(estimator.getDip(Math.toRadians(-65.0),
                Math.toRadians(-88.0), 39e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-58.54, Math.toDegrees(estimator.getDip(Math.toRadians(-23.0),
                Math.toRadians(81.0), 27e3, 2023.5)), ANGLE_ERROR);
        assertEquals(46.72, Math.toDegrees(estimator.getDip(Math.toRadians(34.0),
                0.0, 11e3, 2023.5)), ANGLE_ERROR);
        assertEquals(-68.39, Math.toDegrees(estimator.getDip(Math.toRadians(-62.0),
                Math.toRadians(65.0), 72e3, 2023.5)), ANGLE_ERROR);
        assertEquals(87.48, Math.toDegrees(estimator.getDip(Math.toRadians(86.0),
                Math.toRadians(70.0), 55e3, 2023.5)), ANGLE_ERROR);
        assertEquals(43.07, Math.toDegrees(estimator.getDip(Math.toRadians(32.0),
                Math.toRadians(163.0), 59e3, 2023.5)), ANGLE_ERROR);
        assertEquals(61.70, Math.toDegrees(estimator.getDip(Math.toRadians(48.0),
                Math.toRadians(148.0), 65e3, 2023.5)), ANGLE_ERROR);
        assertEquals(44.09, Math.toDegrees(estimator.getDip(Math.toRadians(30.0),
                Math.toRadians(28.0), 95e3, 2023.5)), ANGLE_ERROR);

        assertEquals(-55.00, Math.toDegrees(estimator.getDip(Math.toRadians(-60.0),
                Math.toRadians(-59.0), 95e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-64.60, Math.toDegrees(estimator.getDip(Math.toRadians(-70.0),
                Math.toRadians(42.0), 95e3, 2024.0)), ANGLE_ERROR);
        assertEquals(89.38, Math.toDegrees(estimator.getDip(Math.toRadians(87.0),
                Math.toRadians(-154.0), 50e3, 2024.0)), ANGLE_ERROR);
        assertEquals(45.88, Math.toDegrees(estimator.getDip(Math.toRadians(32.0),
                Math.toRadians(19.0), 58e3, 2024.0)), ANGLE_ERROR);
        assertEquals(45.87, Math.toDegrees(estimator.getDip(Math.toRadians(34.0),
                Math.toRadians(-13.0), 57e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-67.42, Math.toDegrees(estimator.getDip(Math.toRadians(-76.0),
                Math.toRadians(49.0), 38e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-71.40, Math.toDegrees(estimator.getDip(Math.toRadians(-50.0),
                Math.toRadians(-179.0), 49e3, 2024.0)), ANGLE_ERROR);
        assertEquals(-72.91, Math.toDegrees(estimator.getDip(Math.toRadians(-55.0),
                Math.toRadians(-171.0), 90e3, 2024.0)), ANGLE_ERROR);
        assertEquals(56.60, Math.toDegrees(estimator.getDip(Math.toRadians(42.0),
                Math.toRadians(-19.0), 41e3, 2024.0)), ANGLE_ERROR);
        assertEquals(61.06, Math.toDegrees(estimator.getDip(Math.toRadians(46.0),
                Math.toRadians(-22.0), 19e3, 2024.0)), ANGLE_ERROR);

        assertEquals(31.28, Math.toDegrees(estimator.getDip(Math.toRadians(13.0),
                Math.toRadians(-132.0), 31e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-17.65, Math.toDegrees(estimator.getDip(Math.toRadians(-2.0),
                Math.toRadians(158.0), 93e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-66.29, Math.toDegrees(estimator.getDip(Math.toRadians(-76.0),
                Math.toRadians(40.0), 51e3, 2024.5)), ANGLE_ERROR);
        assertEquals(43.71, Math.toDegrees(estimator.getDip(Math.toRadians(22.0),
                Math.toRadians(-132.0), 64e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-65.69, Math.toDegrees(estimator.getDip(Math.toRadians(-65.0),
                Math.toRadians(55.0), 26e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-57.06, Math.toDegrees(estimator.getDip(Math.toRadians(-21.0),
                Math.toRadians(32.0), 66e3, 2024.5)), ANGLE_ERROR);
        assertEquals(15.77, Math.toDegrees(estimator.getDip(Math.toRadians(9.0),
                Math.toRadians(-172.0), 18e3, 2024.5)), ANGLE_ERROR);
        assertEquals(87.34, Math.toDegrees(estimator.getDip(Math.toRadians(88.0),
                Math.toRadians(26.0), 63e3, 2024.5)), ANGLE_ERROR);
        assertEquals(13.64, Math.toDegrees(estimator.getDip(Math.toRadians(17.0),
                Math.toRadians(5.0), 33e3, 2024.5)), ANGLE_ERROR);
        assertEquals(-47.61, Math.toDegrees(estimator.getDip(Math.toRadians(-18.0),
                Math.toRadians(138.0), 77e3, 2024.5)), ANGLE_ERROR);
    }

    @Test
    void testDipModel2025() throws IOException {
        // test values correspond to I (Deg) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(83.21, Math.toDegrees(estimator.getDip(Math.toRadians(80.0),
                Math.toRadians(0.0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(-14.93, Math.toDegrees(estimator.getDip(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(-72.0, Math.toDegrees(estimator.getDip(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2025.0)), ANGLE_ERROR);
        assertEquals(83.26, Math.toDegrees(estimator.getDip(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2025.0)), ANGLE_ERROR);
        assertEquals(-15.08, Math.toDegrees(estimator.getDip(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2025.0)), ANGLE_ERROR);
        assertEquals(-72.19, Math.toDegrees(estimator.getDip(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2025.0)), ANGLE_ERROR);

        assertEquals(83.24, Math.toDegrees(estimator.getDip(Math.toRadians(80.0),
                Math.toRadians(0.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(-14.65, Math.toDegrees(estimator.getDip(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(-71.92, Math.toDegrees(estimator.getDip(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2027.5)), ANGLE_ERROR);
        assertEquals(83.29, Math.toDegrees(estimator.getDip(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2027.5)), ANGLE_ERROR);
        assertEquals(-14.81, Math.toDegrees(estimator.getDip(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2027.5)), ANGLE_ERROR);
        assertEquals(-72.10, Math.toDegrees(estimator.getDip(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2027.5)), ANGLE_ERROR);
    }

    @Test
    void testHorizontalIntensityModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(1489.24, estimator.getHorizontalIntensity(Math.toRadians(89.0),
                Math.toRadians(-121.0), 28e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1901.61, estimator.getHorizontalIntensity(Math.toRadians(80.0),
                Math.toRadians(-96.0), 48e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2479.65, estimator.getHorizontalIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0), 54e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24320.49, estimator.getHorizontalIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0), 65e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(21689.41, estimator.getHorizontalIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0), 51e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14922.65, estimator.getHorizontalIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0), 39e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22316.83, estimator.getHorizontalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0), 3e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24394.27, estimator.getHorizontalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0), 94e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34949.80, estimator.getHorizontalIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0), 66e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29326.16, estimator.getHorizontalIntensity(
                0.0, Math.toRadians(21.0), 18e3, 2020.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(25525.61, estimator.getHorizontalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0), 6e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34702.32, estimator.getHorizontalIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0), 63e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23267.06, estimator.getHorizontalIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0), 69e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16613.84, estimator.getHorizontalIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20298.75, estimator.getHorizontalIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18105.35, estimator.getHorizontalIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(21686.95, estimator.getHorizontalIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0), 22e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29301.82, estimator.getHorizontalIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0), 40e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23887.76, estimator.getHorizontalIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0), 44e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18357.80, estimator.getHorizontalIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(14297.56, estimator.getHorizontalIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0), 74e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(26841.07, estimator.getHorizontalIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0), 46e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34446.60, estimator.getHorizontalIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0), 69e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31139.48, estimator.getHorizontalIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0), 33e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18472.38, estimator.getHorizontalIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0), 47e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33254.98, estimator.getHorizontalIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(3000.94, estimator.getHorizontalIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0), 83e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14091.82, estimator.getHorizontalIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0), 82e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19958.76, estimator.getHorizontalIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0), 34e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17885.94, estimator.getHorizontalIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0), 56e3, 2021.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(39321.48, estimator.getHorizontalIntensity(0.0,
                Math.toRadians(80.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18553.84, estimator.getHorizontalIntensity(Math.toRadians(-82.0),
                Math.toRadians(-68.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14456.79, estimator.getHorizontalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0), 44e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35900.94, estimator.getHorizontalIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0), 43e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39302.87, estimator.getHorizontalIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0), 64e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24877.86, estimator.getHorizontalIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(12989.03, estimator.getHorizontalIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24827.20, estimator.getHorizontalIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32631.44, estimator.getHorizontalIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0), 19e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33188.54, estimator.getHorizontalIntensity(Math.toRadians(-11.0),
                Math.toRadians(167.0), 86e3, 2021.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(17170.34, estimator.getHorizontalIntensity(Math.toRadians(-66.0),
                Math.toRadians(-5.0), 37e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4696.01, estimator.getHorizontalIntensity(Math.toRadians(72.0),
                Math.toRadians(-115.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28853.48, estimator.getHorizontalIntensity(Math.toRadians(22.0),
                Math.toRadians(174.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20638.93, estimator.getHorizontalIntensity(Math.toRadians(54.0),
                Math.toRadians(178.0), 54e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16773.57, estimator.getHorizontalIntensity(Math.toRadians(-43.0),
                Math.toRadians(50.0), 57e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22662.77, estimator.getHorizontalIntensity(Math.toRadians(-43.0),
                Math.toRadians(-111.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(11584.97, estimator.getHorizontalIntensity(Math.toRadians(-63.0),
                Math.toRadians(178.0), 12e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(26202.78, estimator.getHorizontalIntensity(Math.toRadians(27.0),
                Math.toRadians(-169.0), 38e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10588.73, estimator.getHorizontalIntensity(Math.toRadians(59.0),
                Math.toRadians(-77.0), 61e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13068.78, estimator.getHorizontalIntensity(Math.toRadians(-47.0),
                Math.toRadians(-32.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(13062.66, estimator.getHorizontalIntensity(Math.toRadians(62.0),
                Math.toRadians(53.0), 8e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17288.51, estimator.getHorizontalIntensity(Math.toRadians(-68.0),
                Math.toRadians(-7.0), 77e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33916.26, estimator.getHorizontalIntensity(Math.toRadians(-5.0),
                Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24667.76, estimator.getHorizontalIntensity(Math.toRadians(-29.0),
                Math.toRadians(-107.0), 34e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32962.51, estimator.getHorizontalIntensity(Math.toRadians(27.0),
                Math.toRadians(65.0), 60e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13351.76, estimator.getHorizontalIntensity(Math.toRadians(-72.0),
                Math.toRadians(95.0), 73e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20181.23, estimator.getHorizontalIntensity(Math.toRadians(-46.0),
                Math.toRadians(-85.0), 96e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22748.40, estimator.getHorizontalIntensity(Math.toRadians(-13.0),
                Math.toRadians(-59.0), 0e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13833.66, estimator.getHorizontalIntensity(Math.toRadians(66.0),
                Math.toRadians(-178.0), 16e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16660.39, estimator.getHorizontalIntensity(Math.toRadians(-87.0),
                Math.toRadians(38.0), 72e3, 2022.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(30203.82, estimator.getHorizontalIntensity(Math.toRadians(20.0),
                Math.toRadians(167.0), 49e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28416.81, estimator.getHorizontalIntensity(Math.toRadians(5.0),
                Math.toRadians(-13.0), 71e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36788.89, estimator.getHorizontalIntensity(Math.toRadians(14.0),
                Math.toRadians(65.0), 95e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16885.51, estimator.getHorizontalIntensity(Math.toRadians(-85.0),
                Math.toRadians(-79.0), 86e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17748.96, estimator.getHorizontalIntensity(Math.toRadians(-36.0),
                Math.toRadians(-64.0), 30e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2696.26, estimator.getHorizontalIntensity(Math.toRadians(79.0),
                Math.toRadians(125.0), 75e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28601.69, estimator.getHorizontalIntensity(Math.toRadians(6.0),
                Math.toRadians(-32.0), 21e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19712.46, estimator.getHorizontalIntensity(Math.toRadians(-76.0),
                Math.toRadians(-75.0), 1e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14211.29, estimator.getHorizontalIntensity(Math.toRadians(-46.0),
                Math.toRadians(-41.0), 45e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13992.67, estimator.getHorizontalIntensity(Math.toRadians(-22.0),
                Math.toRadians(-21), 11e3, 2023.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(15183.36, estimator.getHorizontalIntensity(Math.toRadians(54.0),
                Math.toRadians(-120.0), 28e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9228.26, estimator.getHorizontalIntensity(Math.toRadians(-58.0),
                Math.toRadians(156.0), 68e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20793.19, estimator.getHorizontalIntensity(Math.toRadians(-65.0),
                Math.toRadians(-88.0), 39e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(25533.82, estimator.getHorizontalIntensity(Math.toRadians(-23.0),
                Math.toRadians(81.0), 27e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29013.67, estimator.getHorizontalIntensity(Math.toRadians(34.0),
                0.0, 11e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17476.39, estimator.getHorizontalIntensity(Math.toRadians(-62.0),
                Math.toRadians(65.0), 72e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2452.16, estimator.getHorizontalIntensity(Math.toRadians(86.0),
                Math.toRadians(70.0), 55e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28152.41, estimator.getHorizontalIntensity(Math.toRadians(32.0),
                Math.toRadians(163.0), 59e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23671.22, estimator.getHorizontalIntensity(Math.toRadians(48.0),
                Math.toRadians(148.0), 65e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29744.79, estimator.getHorizontalIntensity(Math.toRadians(30.0),
                Math.toRadians(28.0), 95e3, 2023.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(18344.24, estimator.getHorizontalIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18186.37, estimator.getHorizontalIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(605.0402581120115, estimator.getHorizontalIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0), 50e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29382.38, estimator.getHorizontalIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0), 58e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28153.87, estimator.getHorizontalIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0), 57e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18414.65, estimator.getHorizontalIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0), 38e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18107.00, estimator.getHorizontalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0), 49e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16404.29, estimator.getHorizontalIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0), 90e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24382.98, estimator.getHorizontalIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0), 41e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22508.38, estimator.getHorizontalIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0), 19e3, 2024.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(28433.21, estimator.getHorizontalIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0), 31e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34108.06, estimator.getHorizontalIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0), 93e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18518.30, estimator.getHorizontalIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0), 51e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(26293.20, estimator.getHorizontalIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0), 64e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18690.95, estimator.getHorizontalIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0), 26e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15914.15, estimator.getHorizontalIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31008.20, estimator.getHorizontalIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0), 18e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2563.25, estimator.getHorizontalIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0), 63e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34011.43, estimator.getHorizontalIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0), 33e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31843.29, estimator.getHorizontalIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0), 77e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testHorizontalIntensityModel2025() throws IOException {
        // test values correspond to H (nT) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(6523.2, estimator.getHorizontalIntensity(Math.toRadians(80.0),
                Math.toRadians(0.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39677.9, estimator.getHorizontalIntensity(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16898.1, estimator.getHorizontalIntensity(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6216.7, estimator.getHorizontalIntensity(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(37688.7, estimator.getHorizontalIntensity(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15917.1, estimator.getHorizontalIntensity(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(6507.5, estimator.getHorizontalIntensity(Math.toRadians(80.0),
                Math.toRadians(0.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39702.0, estimator.getHorizontalIntensity(Math.toRadians(0.0),
                Math.toRadians(120.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16908.3, estimator.getHorizontalIntensity(Math.toRadians(-80.0),
                Math.toRadians(240.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6201.1, estimator.getHorizontalIntensity(Math.toRadians(80.0),
                Math.toRadians(0.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(37711.8, estimator.getHorizontalIntensity(Math.toRadians(0.0),
                Math.toRadians(120.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15927.0, estimator.getHorizontalIntensity(Math.toRadians(-80.0),
                Math.toRadians(240.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testNorthIntensityModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-569.01, estimator.getNorthIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0), 28e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1518.0, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0), 48e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1568.00, estimator.getNorthIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0), 54e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24319.18, estimator.getNorthIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0), 65e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(21576.68, estimator.getNorthIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0), 51e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14358.57, estimator.getNorthIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8), 39e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19683.62, estimator.getNorthIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0), 3e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23473.45, estimator.getNorthIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0), 94e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34949.76, estimator.getNorthIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0), 66e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29320.37, estimator.getNorthIntensity(
                0.0, Math.toRadians(21.0), 18e3, 2020.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(23963.00, estimator.getNorthIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0), 6e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34701.41, estimator.getNorthIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0), 63e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22628.77, estimator.getNorthIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0), 69e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8953.37, estimator.getNorthIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19570.30, estimator.getNorthIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15257.10, estimator.getNorthIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(21411.62, estimator.getNorthIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0), 22e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28782.41, estimator.getNorthIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0), 40e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23410.32, estimator.getNorthIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0), 44e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16114.64, estimator.getNorthIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(13226.48, estimator.getNorthIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0), 74e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(26042.82, estimator.getNorthIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0), 46e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34440.16, estimator.getNorthIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0), 69e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30690.47, estimator.getNorthIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0), 33e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18368.53, estimator.getNorthIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0), 47e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33239.63, estimator.getNorthIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2405.14, estimator.getNorthIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0), 83e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2249.69, estimator.getNorthIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0), 82e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19341.10, estimator.getNorthIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0), 34e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9201.77, estimator.getNorthIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0), 56e3, 2021.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(39253.06, estimator.getNorthIntensity(
                0.0, Math.toRadians(80.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16016.96, estimator.getNorthIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14163.87, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0), 44e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35892.47, estimator.getNorthIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0), 43e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39285.94, estimator.getNorthIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0), 64e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24298.62, estimator.getNorthIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-9400.74, estimator.getNorthIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23596.33, estimator.getNorthIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32608.56, estimator.getNorthIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0), 19e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32667.21, estimator.getNorthIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0), 86e3, 2021.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(16420.02, estimator.getNorthIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0), 37e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4527.01, estimator.getNorthIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28663.07, estimator.getNorthIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20632.06, estimator.getNorthIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0), 54e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(11344.45, estimator.getNorthIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0), 57e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(20648.98, estimator.getNorthIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6292.51, estimator.getNorthIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0), 12e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(25895.94, estimator.getNorthIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0), 38e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10089.60, estimator.getNorthIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0), 61e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(12675.72, estimator.getNorthIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(12358.70, estimator.getNorthIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0), 8e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(16622.96, estimator.getNorthIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0), 77e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33600.90, estimator.getNorthIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23744.08, estimator.getNorthIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0), 34e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32946.83, estimator.getNorthIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0), 60e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2655.06, estimator.getNorthIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0), 73e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(19151.99, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0), 96e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(21796.35, estimator.getNorthIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0), 0e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13825.18, estimator.getNorthIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0), 16e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(7121.06, estimator.getNorthIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0), 72e3, 2022.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(30079.35, estimator.getNorthIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0), 49e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28186.31, estimator.getNorthIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0), 71e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36787.06, estimator.getNorthIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0), 95e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(12608.27, estimator.getNorthIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0), 86e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17708.76, estimator.getNorthIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0), 30e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2613.59, estimator.getNorthIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0), 75e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(27598.37, estimator.getNorthIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0), 21e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17019.42, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0), 1e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13905.21, estimator.getNorthIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0), 45e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(12773.91, estimator.getNorthIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0), 11e3, 2023.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(14582.49, estimator.getNorthIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0), 28e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(7020.10, estimator.getNorthIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0), 68e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(18039.41, estimator.getNorthIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0), 39e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24783.62, estimator.getNorthIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0), 27e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29009.12, estimator.getNorthIntensity(
                Math.toRadians(34), 0.0, 11e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6831.97, estimator.getNorthIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0), 72e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1183.45, estimator.getNorthIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0), 55e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28151.82, estimator.getNorthIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0), 59e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23356.90, estimator.getNorthIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0), 65e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29657.60, estimator.getNorthIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0), 95e3, 2023.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(18126.12, estimator.getNorthIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10621.33, estimator.getNorthIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(49.86, estimator.getNorthIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0), 50e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29316.97, estimator.getNorthIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0), 58e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28123.39, estimator.getNorthIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0), 57e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8223.09, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0), 38e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15429.70, estimator.getNorthIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0), 49e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(12916.12, estimator.getNorthIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0), 90e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24288.90, estimator.getNorthIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0), 41e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22357.56, estimator.getNorthIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0), 19e3, 2024.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(28064.99, estimator.getNorthIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0), 31e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33843.26, estimator.getNorthIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0), 93e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10463.37, estimator.getNorthIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0), 51e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(25851.87, estimator.getNorthIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0), 64e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8605.94, estimator.getNorthIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0), 26e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15466.32, estimator.getNorthIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15466.32, estimator.getNorthIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30605.55, estimator.getNorthIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0), 18e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2226.75, estimator.getNorthIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0), 63e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34010.13, estimator.getNorthIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0), 33e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31739.14, estimator.getNorthIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0), 77e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testNorthIntensityModel2025() throws IOException {
        // test values correspond to X (nT) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(6521.6, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39677.8, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6117.5, estimator.getNorthIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6216, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(37688.6, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5907.6, estimator.getNorthIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(6500.8, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39701.6, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6200.7, estimator.getNorthIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6196.7, estimator.getNorthIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(37711.5, estimator.getNorthIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5984, estimator.getNorthIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testEastIntensityModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(-1376.25, estimator.getEastIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0), 28e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1145.34, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0), 48e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1920.96, estimator.getEastIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0), 54e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(252.70, estimator.getEastIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0), 65e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2208.53, estimator.getEastIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0), 51e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-4064.10, estimator.getEastIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0), 39e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10516.47, estimator.getEastIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0), 3e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6639.09, estimator.getEastIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0), 94e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(51.82, estimator.getEastIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0), 66e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(582.75, estimator.getEastIntensity(
                0.0, Math.toRadians(21.0), 18e3, 2020.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(8793.81, estimator.getEastIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0), 6e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(251.27, estimator.getEastIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0), 63e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5412.47, estimator.getEastIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0), 69e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(13994.89, estimator.getEastIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5389.09, estimator.getEastIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-9748.05, estimator.getEastIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(3444.75, estimator.getEastIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0), 22e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5492.65, estimator.getEastIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0), 40e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4752.05, estimator.getEastIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0), 44e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8793.58, estimator.getEastIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-5429.58, estimator.getEastIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0), 74e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6497.29, estimator.getEastIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0), 46e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(665.73, estimator.getEastIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0), 69e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5268.98, estimator.getEastIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0), 33e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1955.94, estimator.getEastIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0), 47e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1010.24, estimator.getEastIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1794.70, estimator.getEastIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0), 83e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-13911.09, estimator.getEastIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0), 82e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-4926.86, estimator.getEastIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0), 34e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-15337.35, estimator.getEastIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0), 56e3, 2021.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-2318.56, estimator.getEastIntensity(
                0.0, Math.toRadians(80.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9364.94, estimator.getEastIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2895.40, estimator.getEastIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0), 44e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(779.64, estimator.getEastIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0), 43e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1153.73, estimator.getEastIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0), 64e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5337.13, estimator.getEastIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-8963.31, estimator.getEastIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(7720.32, estimator.getEastIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1221.67, estimator.getEastIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0), 19e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5859.42, estimator.getEastIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0), 86e3, 2021.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-5020.29, estimator.getEastIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0), 37e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1248.48, estimator.getEastIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(3309.29, estimator.getEastIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(532.41, estimator.getEastIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0), 54e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-12355.403556651096, estimator.getEastIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0), 57e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9339.20, estimator.getEastIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9727.06, estimator.getEastIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0), 12e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(3998.23, estimator.getEastIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0), 38e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3212.65, estimator.getEastIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0), 61e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3181.04, estimator.getEastIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(4230.32, estimator.getEastIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0), 8e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-4750.78, estimator.getEastIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0), 77e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4614.30, estimator.getEastIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6687.08, estimator.getEastIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0), 34e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1016.59, estimator.getEastIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0), 60e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-13085.11, estimator.getEastIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0), 73e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(6362.67, estimator.getEastIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0), 96e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-6512.21, estimator.getEastIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0), 0e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(484.23, estimator.getEastIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0), 16e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-15061.84, estimator.getEastIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0), 72e3, 2022.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(2739.25, estimator.getEastIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0), 49e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3612.05, estimator.getEastIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0), 71e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-367.05, estimator.getEastIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0), 95e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(11231.74, estimator.getEastIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0), 86e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1193.97, estimator.getEastIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0), 30e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-662.55, estimator.getEastIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0), 75e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-7509.10, estimator.getEastIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0), 21e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9945.85, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0), 1e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2933.59, estimator.getEastIntensity(
                Math.toRadians(-46), Math.toRadians(-41.0), 45e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-5711.57, estimator.getEastIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0), 11e3, 2023.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(4229.11, estimator.getEastIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0), 28e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5989.91, estimator.getEastIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0), 68e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10341.00, estimator.getEastIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0), 39e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-6143.93, estimator.getEastIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0), 27e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(513.86, estimator.getEastIntensity(
                Math.toRadians(34.0), 0.0, 11e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-16085.66, estimator.getEastIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0), 72e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2147.68, estimator.getEastIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0), 55e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(181.91, estimator.getEastIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0), 59e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3844.69, estimator.getEastIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0), 65e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2275.91, estimator.getEastIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0), 95e3, 2023.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(2820.44, estimator.getEastIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-14762.49, estimator.getEastIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-602.98, estimator.getEastIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0), 50e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1959.46, estimator.getEastIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0), 58e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1309.64, estimator.getEastIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0), 57e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-16476.65, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0), 38e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(9475.63, estimator.getEastIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0), 49e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(10113.08, estimator.getEastIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0), 90e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2139.95, estimator.getEastIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0), 41e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-2601.27, estimator.getEastIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0), 19e3, 2024.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(4561.08, estimator.getEastIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0), 31e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4241.88, estimator.getEastIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0), 93e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-15278.92, estimator.getEastIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0), 51e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4797.22, estimator.getEastIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0), 64e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-16591.85, estimator.getEastIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0), 26e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3748.76, estimator.getEastIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3748.76, estimator.getEastIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(4980.85, estimator.getEastIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0), 18e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(1269.59, estimator.getEastIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0), 63e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(297.72, estimator.getEastIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0), 33e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(2573.27, estimator.getEastIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0), 77e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testEastIntensityModel2025() throws IOException {
        // test values correspond to Y (nT) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(145.9, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-109.6, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15751.9, estimator.getEastIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(92.4, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-96.2, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14780.3, estimator.getEastIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(294.5, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-167.4, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15730.3, estimator.getEastIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(233.8, estimator.getEastIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-148.7, estimator.getEastIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(14760.1, estimator.getEastIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);

    }

    @Test
    void testVerticalIntensityModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(56103.91, estimator.getVerticalIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0), 28e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55683.79, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0), 48e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(56535.10, estimator.getVerticalIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0), 54e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(49735.93, estimator.getVerticalIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0), 65e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-52693.33, estimator.getVerticalIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0), 51e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-24663.85, estimator.getVerticalIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0), 39e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-31882.32, estimator.getVerticalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0), 3e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-19323.70, estimator.getVerticalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0), 94e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(7966.37, estimator.getVerticalIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0), 66e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-14589.63, estimator.getVerticalIntensity(
                0.0, Math.toRadians(21.0), 18e3, 2020.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-32882.36, estimator.getVerticalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0), 6e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30126.44, estimator.getVerticalIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0), 63e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35836.61, estimator.getVerticalIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0), 69e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-51643.35, estimator.getVerticalIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-23756.54, estimator.getVerticalIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-31054.04, estimator.getVerticalIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-55450.86, estimator.getVerticalIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0), 22e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-8189.87, estimator.getVerticalIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0), 40e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38220.58, estimator.getVerticalIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0), 44e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-44787.10, estimator.getVerticalIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-23851.09, estimator.getVerticalIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0), 74e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-18285.57, estimator.getVerticalIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0), 46e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24908.11, estimator.getVerticalIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0), 69e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-1253.42, estimator.getVerticalIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0), 33e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-33661.73, estimator.getVerticalIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0), 47e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-33327.91, estimator.getVerticalIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54201.54, estimator.getVerticalIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0), 83e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-53540.40, estimator.getVerticalIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0), 82e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-25962.84, estimator.getVerticalIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0), 34e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-45481.66, estimator.getVerticalIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0), 56e3, 2021.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-12216.61, estimator.getVerticalIntensity(
                0.0, Math.toRadians(80.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-46323.93, estimator.getVerticalIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-19752.42, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0), 44e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(15883.37, estimator.getVerticalIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0), 43e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(5122.25, estimator.getVerticalIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0), 64e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32428.02, estimator.getVerticalIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-58289.44, estimator.getVerticalIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-24157.40, estimator.getVerticalIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33769.68, estimator.getVerticalIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0), 19e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-20316.11, estimator.getVerticalIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0), 86e3, 2021.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-28851.72, estimator.getVerticalIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0), 37e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55944.06, estimator.getVerticalIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(17953.03, estimator.getVerticalIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(45057.46, estimator.getVerticalIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0), 54e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-32859.78, estimator.getVerticalIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0), 57e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-29740.46, estimator.getVerticalIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-61422.27, estimator.getVerticalIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0), 12e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24091.37, estimator.getVerticalIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0), 38e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54735.93, estimator.getVerticalIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0), 61e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-20588.43, estimator.getVerticalIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(54479.79, estimator.getVerticalIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0), 8e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-29918.88, estimator.getVerticalIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0), 77e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-14424.69, estimator.getVerticalIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-14424.69, estimator.getVerticalIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-18996.17, estimator.getVerticalIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0), 34e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30499.46, estimator.getVerticalIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0), 60e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-55430.18, estimator.getVerticalIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0), 73e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-21899.93, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0), 96e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-5396.18, estimator.getVerticalIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0), 0e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54058.15, estimator.getVerticalIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0), 16e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-48579.53, estimator.getVerticalIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0), 72e3, 2022.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(15309.25, estimator.getVerticalIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0), 49e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-8846.18, estimator.getVerticalIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0), 71e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(11513.54, estimator.getVerticalIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0), 95e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-47353.39, estimator.getVerticalIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0), 86e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-14536.66, estimator.getVerticalIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0), 30e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(57113.34, estimator.getVerticalIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0), 75e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-3582.78, estimator.getVerticalIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0), 21e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-42884.71, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0), 1e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-19832.07, estimator.getVerticalIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0), 45e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-21293.83, estimator.getVerticalIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0), 11e3, 2023.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(52950.92, estimator.getVerticalIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0), 28e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-62443.54, estimator.getVerticalIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0), 68e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-36408.46, estimator.getVerticalIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0), 39e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-41730.68, estimator.getVerticalIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0), 27e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30807.14, estimator.getVerticalIntensity(
                Math.toRadians(34.0), 0.0, 11e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-44113.52, estimator.getVerticalIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0), 72e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55764.08, estimator.getVerticalIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0), 55e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(26321.18, estimator.getVerticalIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0), 59e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(43956.00, estimator.getVerticalIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0), 65e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28817.59, estimator.getVerticalIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0), 95e3, 2023.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(-26197.70, estimator.getVerticalIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-38308.95, estimator.getVerticalIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55925.28, estimator.getVerticalIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0), 50e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30302.26, estimator.getVerticalIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0), 58e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29024.48, estimator.getVerticalIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0), 57e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-44287.00, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0), 38e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-53792.43, estimator.getVerticalIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0), 49e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-53346.86, estimator.getVerticalIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0), 90e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36984.14, estimator.getVerticalIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0), 41e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(40713.44, estimator.getVerticalIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0), 19e3, 2024.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(17276.28, estimator.getVerticalIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0), 31e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-10851.77, estimator.getVerticalIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0), 93e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-42172.82, estimator.getVerticalIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0), 51e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(25136.44, estimator.getVerticalIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0), 64e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-41367.58, estimator.getVerticalIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0), 26e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-24565.80, estimator.getVerticalIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8755.23, estimator.getVerticalIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0), 18e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55161.20, estimator.getVerticalIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0), 63e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(8252.36, estimator.getVerticalIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0), 33e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-34888.71, estimator.getVerticalIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0), 77e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testVerticalIntensityModel2025() throws IOException {
        // test values correspond to Z (nT) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(54791.5, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-10580.2, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-52022.5, estimator.getVerticalIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(52598.8, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-10152.1, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-49540.7, estimator.getVerticalIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(54869.4, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-10381.8, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-51783.7, estimator.getVerticalIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(52670.5, estimator.getVerticalIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-9969.8, estimator.getVerticalIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(-49317.7, estimator.getVerticalIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testIntensityModel2020() throws IOException {
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(56123.67, estimator.getIntensity(
                Math.toRadians(89.0), Math.toRadians(-121.0), 28e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55716.25, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(-96.0), 48e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(56589.45, estimator.getIntensity(
                Math.toRadians(82.0), Math.toRadians(87.0), 54e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55363.83, estimator.getIntensity(
                Math.toRadians(43.0), Math.toRadians(93.0), 65e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(56982.61, estimator.getIntensity(
                Math.toRadians(-33.0), Math.toRadians(109.0), 51e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28826.91, estimator.getIntensity(
                Math.toRadians(-59.0), Math.toRadians(-8.0), 39e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38916.88, estimator.getIntensity(
                Math.toRadians(-50.0), Math.toRadians(-103.0), 3e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31120.50, estimator.getIntensity(
                Math.toRadians(-29.0), Math.toRadians(-110.0), 94e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35846.22, estimator.getIntensity(
                Math.toRadians(14.0), Math.toRadians(143.0), 66e3, 2020.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32754.86, estimator.getIntensity(
                0.0, Math.toRadians(21.0), 18e3, 2020.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(41626.99, estimator.getIntensity(
                Math.toRadians(-36.0), Math.toRadians(-137.0), 6e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(45954.90, estimator.getIntensity(
                Math.toRadians(26.0), Math.toRadians(81.0), 63e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(42727.26, estimator.getIntensity(
                Math.toRadians(38.0), Math.toRadians(-144.0), 69e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54249.94, estimator.getIntensity(
                Math.toRadians(-70.0), Math.toRadians(-133.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31247.60, estimator.getIntensity(
                Math.toRadians(-52.0), Math.toRadians(-75.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35946.59, estimator.getIntensity(
                Math.toRadians(-66.0), Math.toRadians(17.0), 8e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(59540.92, estimator.getIntensity(
                Math.toRadians(-37.0), Math.toRadians(140.0), 22e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(30424.84, estimator.getIntensity(
                Math.toRadians(-12.0), Math.toRadians(-129.0), 40e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(45071.48, estimator.getIntensity(
                Math.toRadians(33.0), Math.toRadians(-118.0), 44e3, 2020.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(48403.44, estimator.getIntensity(
                Math.toRadians(-81.0), Math.toRadians(-67.0), 50e3, 2020.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(27808.18, estimator.getIntensity(
                Math.toRadians(-57.0), Math.toRadians(3.0), 74e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32477.77, estimator.getIntensity(
                Math.toRadians(-24.0), Math.toRadians(-122.0), 46e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(42508.61, estimator.getIntensity(
                Math.toRadians(23.0), Math.toRadians(63.0), 69e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31164.69, estimator.getIntensity(
                Math.toRadians(-3.0), Math.toRadians(-147.0), 33e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38397.15, estimator.getIntensity(
                Math.toRadians(-72.0), Math.toRadians(-22.0), 47e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47081.24, estimator.getIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47081.24, estimator.getIntensity(
                Math.toRadians(-14.0), Math.toRadians(99.0), 62e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54284.56, estimator.getIntensity(
                Math.toRadians(86.0), Math.toRadians(-46.0), 83e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55363.83, estimator.getIntensity(
                Math.toRadians(-64.0), Math.toRadians(87.0), 82e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32747.84, estimator.getIntensity(
                Math.toRadians(-19.0), Math.toRadians(43.0), 34e3, 2021.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(48872.16, estimator.getIntensity(
                Math.toRadians(-81.0), Math.toRadians(40.0), 56e3, 2021.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(41175.53, estimator.getIntensity(
                0.0, Math.toRadians(80.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(49901.42, estimator.getIntensity(
                Math.toRadians(-82.0), Math.toRadians(-68.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24477.68, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-42.0), 44e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39257.59, estimator.getIntensity(
                Math.toRadians(17.0), Math.toRadians(52.0), 43e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39635.25, estimator.getIntensity(
                Math.toRadians(10.0), Math.toRadians(78.0), 64e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(40871.56, estimator.getIntensity(
                Math.toRadians(33.0), Math.toRadians(-145.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(59719.13, estimator.getIntensity(
                Math.toRadians(-79.0), Math.toRadians(115.0), 12e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34640.58, estimator.getIntensity(
                Math.toRadians(-33.0), Math.toRadians(-114.0), 14e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(46959.58, estimator.getIntensity(
                Math.toRadians(29.0), Math.toRadians(66.0), 19e3, 2021.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38913.03, estimator.getIntensity(
                Math.toRadians(-11.0), Math.toRadians(167.0), 86e3, 2021.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(33574.43, estimator.getIntensity(
                Math.toRadians(-66.0), Math.toRadians(-5.0), 37e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(56140.81, estimator.getIntensity(
                Math.toRadians(72.0), Math.toRadians(-115.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(33982.85, estimator.getIntensity(
                Math.toRadians(22.0), Math.toRadians(174.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(49559.46, estimator.getIntensity(
                Math.toRadians(54.0), Math.toRadians(178.0), 54e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36893.33, estimator.getIntensity(
                Math.toRadians(-43.0), Math.toRadians(50.0), 57e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(37391.12, estimator.getIntensity(
                Math.toRadians(-43.0), Math.toRadians(-111.0), 44e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(62505.26, estimator.getIntensity(
                Math.toRadians(-63.0), Math.toRadians(178.0), 12e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35594.66, estimator.getIntensity(
                Math.toRadians(27.0), Math.toRadians(-169.0), 38e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55750.73, estimator.getIntensity(
                Math.toRadians(59.0), Math.toRadians(-77.0), 61e3, 2022.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24385.99, estimator.getIntensity(
                Math.toRadians(-47.0), Math.toRadians(-32.0), 67e3, 2022.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(56023.93, estimator.getIntensity(
                Math.toRadians(62.0), Math.toRadians(53.0), 8e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34554.77, estimator.getIntensity(
                Math.toRadians(-68.0), Math.toRadians(-7.0), 77e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36856.26, estimator.getIntensity(
                Math.toRadians(-5.0), Math.toRadians(159.0), 98e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(31134.43, estimator.getIntensity(
                Math.toRadians(-29.0), Math.toRadians(-107.0), 34e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(44908.18, estimator.getIntensity(
                Math.toRadians(27.0), Math.toRadians(65.0), 60e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(57015.56, estimator.getIntensity(
                Math.toRadians(-72.0), Math.toRadians(95.0), 73e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29780.68, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-85.0), 96e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(23379.66, estimator.getIntensity(
                Math.toRadians(-13.0), Math.toRadians(-59.0), 0e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55800.12, estimator.getIntensity(
                Math.toRadians(66.0), Math.toRadians(-178.0), 16e3, 2022.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(51356.97, estimator.getIntensity(
                Math.toRadians(-87.0), Math.toRadians(38.0), 72e3, 2022.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(33862.13, estimator.getIntensity(
                Math.toRadians(20.0), Math.toRadians(167.0), 49e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29761.89, estimator.getIntensity(
                Math.toRadians(5.0), Math.toRadians(-13.0), 71e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38548.47, estimator.getIntensity(
                Math.toRadians(14.0), Math.toRadians(65.0), 95e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(50273.89, estimator.getIntensity(
                Math.toRadians(-85.0), Math.toRadians(-79.0), 86e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(22942.11, estimator.getIntensity(
                Math.toRadians(-36.0), Math.toRadians(-64.0), 30e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(57176.95, estimator.getIntensity(
                Math.toRadians(79.0), Math.toRadians(125.0), 75e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(28825.22, estimator.getIntensity(
                Math.toRadians(6.0), Math.toRadians(-32.0), 21e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47198.29, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(-75.0), 1e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(24398.19, estimator.getIntensity(
                Math.toRadians(-46.0), Math.toRadians(-41.0), 45e3, 2023.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(25479.83, estimator.getIntensity(
                Math.toRadians(-22.0), Math.toRadians(-21.0), 11e3, 2023.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(55084.80, estimator.getIntensity(
                Math.toRadians(54.0), Math.toRadians(-120.0), 28e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(63121.76, estimator.getIntensity(
                Math.toRadians(-58.0), Math.toRadians(156.0), 68e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(41927.71, estimator.getIntensity(
                Math.toRadians(-65.0), Math.toRadians(-88.0), 39e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(48922.65, estimator.getIntensity(
                Math.toRadians(-23.0), Math.toRadians(81.0), 27e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(42318.71, estimator.getIntensity(
                Math.toRadians(34.0), 0.0, 11e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47449.20, estimator.getIntensity(
                Math.toRadians(-62.0), Math.toRadians(65.0), 72e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55817.97, estimator.getIntensity(
                Math.toRadians(86.0), Math.toRadians(70.0), 55e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(38540.40, estimator.getIntensity(
                Math.toRadians(32.0), Math.toRadians(163.0), 59e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(49924.51, estimator.getIntensity(
                Math.toRadians(48.0), Math.toRadians(148.0), 65e3, 2023.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(41415.05, estimator.getIntensity(
                Math.toRadians(30.0), Math.toRadians(28.0), 95e3, 2023.5) * TO_NANO, INTENSITY_ERROR);

        assertEquals(31981.72, estimator.getIntensity(
                Math.toRadians(-60.0), Math.toRadians(-59.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(42406.60, estimator.getIntensity(
                Math.toRadians(-70.0), Math.toRadians(42.0), 95e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55928.55, estimator.getIntensity(
                Math.toRadians(87.0), Math.toRadians(-154.0), 50e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(42208.43, estimator.getIntensity(
                Math.toRadians(32.0), Math.toRadians(19.0), 58e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(40435.88, estimator.getIntensity(
                Math.toRadians(34.0), Math.toRadians(-13.0), 57e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47962.88, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(49.0), 38e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(56758.16, estimator.getIntensity(
                Math.toRadians(-50.0), Math.toRadians(-179.0), 49e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55812.08, estimator.getIntensity(
                Math.toRadians(-55.0), Math.toRadians(-171.0), 90e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(44298.50, estimator.getIntensity(
                Math.toRadians(42.0), Math.toRadians(-19.0), 41e3, 2024.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(46521.09, estimator.getIntensity(
                Math.toRadians(46.0), Math.toRadians(-22.0), 19e3, 2024.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(33270.36, estimator.getIntensity(
                Math.toRadians(13.0), Math.toRadians(-132.0), 31e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(35792.75, estimator.getIntensity(
                Math.toRadians(-2.0), Math.toRadians(158.0), 93e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(46059.47, estimator.getIntensity(
                Math.toRadians(-76.0), Math.toRadians(40.0), 51e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(36375.44, estimator.getIntensity(
                Math.toRadians(22.0), Math.toRadians(-132.0), 64e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(45394.15, estimator.getIntensity(
                Math.toRadians(-65.0), Math.toRadians(55.0), 26e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(29270.10, estimator.getIntensity(
                Math.toRadians(-21.0), Math.toRadians(32.0), 66e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(32220.53, estimator.getIntensity(
                Math.toRadians(9.0), Math.toRadians(-172.0), 18e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(55220.72, estimator.getIntensity(
                Math.toRadians(88.0), Math.toRadians(26.0), 63e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(34998.27, estimator.getIntensity(
                Math.toRadians(17.0), Math.toRadians(5.0), 33e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(47235.76, estimator.getIntensity(
                Math.toRadians(-18.0), Math.toRadians(138.0), 77e3, 2024.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testIntensityModel2025() throws IOException {
        // test values correspond to F (nT) column in WMM test values document
        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        assertEquals(55178.5, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(41064.3, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54698.2, estimator.getIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(52964.9, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39032.1, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);
        assertEquals(52035, estimator.getIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2025.0) * TO_NANO, INTENSITY_ERROR);

        assertEquals(55253.9, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(41036.9, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(54474.2, estimator.getIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 0.0, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(53034.3, estimator.getIntensity(
                Math.toRadians(80.0), Math.toRadians(0.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(39007.4, estimator.getIntensity(
                Math.toRadians(0.0), Math.toRadians(120.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
        assertEquals(51825.7, estimator.getIntensity(
                Math.toRadians(-80.0), Math.toRadians(240.0), 100e3, 2027.5) * TO_NANO, INTENSITY_ERROR);
    }

    @Test
    void testConvertTime() {
        final var cal = new GregorianCalendar(2010, Calendar.JANUARY, 1);
        assertEquals(2010.0, WMMEarthMagneticFluxDensityEstimator.convertTime(cal), 0.0);

        // the full day of July 1, 0 hours into 2 July
        final var cal2 = new GregorianCalendar(2012, Calendar.JULY, 2);
        assertTrue(cal2.isLeapYear(2012));
        assertEquals(2012.5, WMMEarthMagneticFluxDensityEstimator.convertTime(cal2), 0.0);

        final var cal3 = new GregorianCalendar(2013, Calendar.APRIL, 14);
        assertFalse(cal3.isLeapYear(2013));
        assertEquals(2013.282, WMMEarthMagneticFluxDensityEstimator.convertTime(cal3), TIME_ERROR);
    }

    @Test
    void testGetDeclinationWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var declination1 = estimator.getDeclination(latitude, longitude);
        final var declination2 = estimator.getDeclination(latitudeAngle, longitudeAngle);

        assertEquals(declination1, declination2, 0.0);

        final var result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, result1);
        final var result2 = estimator.getDeclinationAsAngle(latitude, longitude);
        final var result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, result3);
        final var result4 = estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle);

        assertEquals(declination1, result1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, result1.getUnit());
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
    }

    @Test
    void testGetDeclination() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var declination1 = estimator.getDeclination(latitude, longitude, height, year);
        final var declination2 = estimator.getDeclination(latitude, longitude, height, calendar);
        final var declination3 = estimator.getDeclination(latitude, longitude, height, date);
        final var declination4 = estimator.getDeclination(latitudeAngle, longitudeAngle, heightDistance, year);
        final var declination5 = estimator.getDeclination(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var declination6 = estimator.getDeclination(latitudeAngle, longitudeAngle, heightDistance, date);
        final var declination7 = estimator.getDeclination(position, year);
        final var declination8 = estimator.getDeclination(position, calendar);
        final var declination9 = estimator.getDeclination(position, date);

        assertEquals(declination1, declination2, 0.0);
        assertEquals(declination1, declination3, 0.0);
        assertEquals(declination1, declination4, 0.0);
        assertEquals(declination1, declination5, 0.0);
        assertEquals(declination1, declination6, 0.0);
        assertEquals(declination1, declination7, 0.0);
        assertEquals(declination1, declination8, 0.0);
        assertEquals(declination1, declination9, 0.0);

        final var result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height, year, result1);
        final var result2 = estimator.getDeclinationAsAngle(latitude, longitude, height, year);
        final var result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height, calendar, result3);
        final var result4 = estimator.getDeclinationAsAngle(latitude, longitude, height, calendar);
        final var result5 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitude, longitude, height, date, result5);
        final var result6 = estimator.getDeclinationAsAngle(latitude, longitude, height, date);
        final var result7 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, year, result7);
        final var result8 = estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, year);
        final var result9 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, calendar, result9);
        final var result10 = estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var result11 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, date, result11);
        final var result12 = estimator.getDeclinationAsAngle(latitudeAngle, longitudeAngle, heightDistance, date);
        final var result13 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, year, result13);
        final var result14 = estimator.getDeclinationAsAngle(position, year);
        final var result15 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, calendar, result15);
        final var result16 = estimator.getDeclinationAsAngle(position, calendar);
        final var result17 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDeclinationAsAngle(position, date, result17);
        final var result18 = estimator.getDeclinationAsAngle(position, date);

        assertEquals(result1.getValue().doubleValue(), declination1, 0.0);
        assertEquals(AngleUnit.RADIANS, result1.getUnit());
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
    void testGetDipWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var dip1 = estimator.getDip(latitude, longitude);
        final var dip2 = estimator.getDip(latitudeAngle, longitudeAngle);

        assertEquals(dip1, dip2, 0.0);

        final var result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, result1);
        final var result2 = estimator.getDipAsAngle(latitude, longitude);
        final var result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle, result3);
        final var result4 = estimator.getDipAsAngle(latitudeAngle, longitudeAngle);

        assertEquals(result1.getValue().doubleValue(), dip1, 0.0);
        assertEquals(AngleUnit.RADIANS, result1.getUnit());
        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
    }

    @Test
    void testGetDip() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var dip1 = estimator.getDip(latitude, longitude, height, year);
        final var dip2 = estimator.getDip(latitude, longitude, height, calendar);
        final var dip3 = estimator.getDip(latitude, longitude, height, date);
        final var dip4 = estimator.getDip(latitudeAngle, longitudeAngle, heightDistance, year);
        final var dip5 = estimator.getDip(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var dip6 = estimator.getDip(latitudeAngle, longitudeAngle, heightDistance, date);
        final var dip7 = estimator.getDip(position, year);
        final var dip8 = estimator.getDip(position, calendar);
        final var dip9 = estimator.getDip(position, date);

        assertEquals(dip1, dip2, 0.0);
        assertEquals(dip1, dip3, 0.0);
        assertEquals(dip1, dip4, 0.0);
        assertEquals(dip1, dip5, 0.0);
        assertEquals(dip1, dip6, 0.0);
        assertEquals(dip1, dip7, 0.0);
        assertEquals(dip1, dip8, 0.0);
        assertEquals(dip1, dip9, 0.0);

        final var result1 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height, year, result1);
        final var result2 = estimator.getDipAsAngle(latitude, longitude, height, year);
        final var result3 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height, calendar, result3);
        final var result4 = estimator.getDipAsAngle(latitude, longitude, height, calendar);
        final var result5 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitude, longitude, height, date, result5);
        final var result6 = estimator.getDipAsAngle(latitude, longitude, height, date);
        final var result7 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, year, result7);
        final var result8 = estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, year);
        final var result9 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, calendar, result9);
        final var result10 = estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var result11 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, date, result11);
        final var result12 = estimator.getDipAsAngle(latitudeAngle, longitudeAngle, heightDistance, date);
        final var result13 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, year, result13);
        final var result14 = estimator.getDipAsAngle(position, year);
        final var result15 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, calendar, result15);
        final var result16 = estimator.getDipAsAngle(position, calendar);
        final var result17 = new Angle(0.0, AngleUnit.DEGREES);
        estimator.getDipAsAngle(position, date, result17);
        final var result18 = estimator.getDipAsAngle(position, date);

        assertEquals(dip1, result1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, result1.getUnit());
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
    void testGetIntensityWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getIntensity(latitude, longitude);
        final var intensity2 = estimator.getIntensity(latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    void testGetIntensity() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getIntensity(latitude, longitude, height, year);
        final var intensity2 = estimator.getIntensity(latitude, longitude, height, calendar);
        final var intensity3 = estimator.getIntensity(latitude, longitude, height, date);
        final var intensity4 = estimator.getIntensity(latitudeAngle, longitudeAngle, heightDistance, year);
        final var intensity5 = estimator.getIntensity(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var intensity6 = estimator.getIntensity(latitudeAngle, longitudeAngle, heightDistance, date);
        final var intensity7 = estimator.getIntensity(position, year);
        final var intensity8 = estimator.getIntensity(position, calendar);
        final var intensity9 = estimator.getIntensity(position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    void testGetHorizontalIntensityWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getHorizontalIntensity(latitude, longitude);
        final var intensity2 = estimator.getHorizontalIntensity(latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    void testGetHorizontalIntensity() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getHorizontalIntensity(latitude, longitude, height, year);
        final var intensity2 = estimator.getHorizontalIntensity(latitude, longitude, height, calendar);
        final var intensity3 = estimator.getHorizontalIntensity(latitude, longitude, height, date);
        final var intensity4 = estimator.getHorizontalIntensity(latitudeAngle, longitudeAngle, heightDistance, year);
        final var intensity5 = estimator.getHorizontalIntensity(latitudeAngle, longitudeAngle, heightDistance,
                calendar);
        final var intensity6 = estimator.getHorizontalIntensity(latitudeAngle, longitudeAngle, heightDistance, date);
        final var intensity7 = estimator.getHorizontalIntensity(position, year);
        final var intensity8 = estimator.getHorizontalIntensity(position, calendar);
        final var intensity9 = estimator.getHorizontalIntensity(position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    void testGetVerticalIntensityWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getVerticalIntensity(latitude, longitude);
        final var intensity2 = estimator.getVerticalIntensity(latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    void testGetVerticalIntensity() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getVerticalIntensity(latitude, longitude, height, year);
        final var intensity2 = estimator.getVerticalIntensity(latitude, longitude, height, calendar);
        final var intensity3 = estimator.getVerticalIntensity(latitude, longitude, height, date);
        final var intensity4 = estimator.getVerticalIntensity(latitudeAngle, longitudeAngle, heightDistance, year);
        final var intensity5 = estimator.getVerticalIntensity(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var intensity6 = estimator.getVerticalIntensity(latitudeAngle, longitudeAngle, heightDistance, date);
        final var intensity7 = estimator.getVerticalIntensity(position, year);
        final var intensity8 = estimator.getVerticalIntensity(position, calendar);
        final var intensity9 = estimator.getVerticalIntensity(position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    void testGetNorthIntensityWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getNorthIntensity(latitude, longitude);
        final var intensity2 = estimator.getNorthIntensity(latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    void testGetNorthIntensity() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getNorthIntensity(latitude, longitude, height, year);
        final var intensity2 = estimator.getNorthIntensity(latitude, longitude, height, calendar);
        final var intensity3 = estimator.getNorthIntensity(latitude, longitude, height, date);
        final var intensity4 = estimator.getNorthIntensity(latitudeAngle, longitudeAngle, heightDistance, year);
        final var intensity5 = estimator.getNorthIntensity(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var intensity6 = estimator.getNorthIntensity(latitudeAngle, longitudeAngle, heightDistance, date);
        final var intensity7 = estimator.getNorthIntensity(position, year);
        final var intensity8 = estimator.getNorthIntensity(position, calendar);
        final var intensity9 = estimator.getNorthIntensity(position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    void testGetEastIntensityWithDefaultTimeAndHeight() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getEastIntensity(latitude, longitude);
        final var intensity2 = estimator.getEastIntensity(latitudeAngle, longitudeAngle);

        assertEquals(intensity1, intensity2, 0.0);
    }

    @Test
    void testGetEastIntensity() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var intensity1 = estimator.getEastIntensity(latitude, longitude, height, year);
        final var intensity2 = estimator.getEastIntensity(latitude, longitude, height, calendar);
        final var intensity3 = estimator.getEastIntensity(latitude, longitude, height, date);
        final var intensity4 = estimator.getEastIntensity(latitudeAngle, longitudeAngle, heightDistance, year);
        final var intensity5 = estimator.getEastIntensity(latitudeAngle, longitudeAngle, heightDistance, calendar);
        final var intensity6 = estimator.getEastIntensity(latitudeAngle, longitudeAngle, heightDistance, date);
        final var intensity7 = estimator.getEastIntensity(position, year);
        final var intensity8 = estimator.getEastIntensity(position, calendar);
        final var intensity9 = estimator.getEastIntensity(position, date);

        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity2, 0.0);
        assertEquals(intensity1, intensity3, 0.0);
        assertEquals(intensity1, intensity4, 0.0);
        assertEquals(intensity1, intensity5, 0.0);
        assertEquals(intensity1, intensity6, 0.0);
        assertEquals(intensity1, intensity7, 0.0);
        assertEquals(intensity1, intensity8, 0.0);
        assertEquals(intensity1, intensity9, 0.0);
    }

    @Test
    void testEstimate1() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var bn = estimator.getNorthIntensity(latitude, longitude);
        final var be = estimator.getEastIntensity(latitude, longitude);
        final var bd = estimator.getVerticalIntensity(latitude, longitude);

        final var b1 = new NEDMagneticFluxDensity();
        estimator.estimate(latitude, longitude, b1);

        assertEquals(bn, b1.getBn(), ABSOLUTE_ERROR);
        assertEquals(be, b1.getBe(), ABSOLUTE_ERROR);
        assertEquals(bd, b1.getBd(), ABSOLUTE_ERROR);

        final var b2 = estimator.estimate(latitude, longitude);
        assertEquals(b1, b2);

        final var b3 = new NEDMagneticFluxDensity();
        estimator.estimate(latitudeAngle, longitudeAngle, b3);
        assertEquals(b1, b3);

        final var b4 = estimator.estimate(latitudeAngle, longitudeAngle);
        assertEquals(b1, b4);
    }

    @Test
    void testEstimate2() throws IOException {
        final var position = createPosition();
        final var latitude = position.getLatitude();
        final var longitude = position.getLongitude();
        final var height = position.getHeight();
        final var latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final var longitudeAngle = new Angle(longitude, AngleUnit.RADIANS);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var timestamp = createTimestamp();
        final var date = createDate(timestamp);
        final var calendar = createCalendar(timestamp);
        final var year = createYear(calendar);

        final var estimator = new WMMEarthMagneticFluxDensityEstimator();

        final var bn = estimator.getNorthIntensity(latitude, longitude, height, year);
        final var be = estimator.getEastIntensity(latitude, longitude, height, year);
        final var bd = estimator.getVerticalIntensity(latitude, longitude, height, year);

        final var b1 = new NEDMagneticFluxDensity();
        estimator.estimate(latitude, longitude, height, year, b1);

        assertEquals(bn, b1.getBn(), ABSOLUTE_ERROR);
        assertEquals(be, b1.getBe(), ABSOLUTE_ERROR);
        assertEquals(bd, b1.getBd(), ABSOLUTE_ERROR);

        final var b2 = estimator.estimate(latitude, longitude, height, year);
        assertEquals(b1, b2);

        final var b3 = new NEDMagneticFluxDensity();
        estimator.estimate(latitude, longitude, height, calendar, b3);
        assertEquals(b1, b3);

        final var b4 = estimator.estimate(latitude, longitude, height, calendar);
        assertEquals(b1, b4);

        final var b5 = new NEDMagneticFluxDensity();
        estimator.estimate(latitude, longitude, height, date, b5);
        assertEquals(b1, b5);

        final var b6 = estimator.estimate(latitude, longitude, height, date);
        assertEquals(b1, b6);

        final var b7 = new NEDMagneticFluxDensity();
        estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, year, b7);
        assertEquals(b1, b7);

        final var b8 = estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, year);
        assertEquals(b1, b8);

        final var b9 = new NEDMagneticFluxDensity();
        estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, calendar, b9);
        assertEquals(b1, b9);

        final var b10 = estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, calendar);
        assertEquals(b1, b10);

        final var b11 = new NEDMagneticFluxDensity();
        estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, date, b11);
        assertEquals(b1, b11);

        final var b12 = estimator.estimate(latitudeAngle, longitudeAngle, heightDistance, date);
        assertEquals(b1, b12);

        final var b13 = new NEDMagneticFluxDensity();
        estimator.estimate(position, year, b13);
        assertEquals(b1, b13);

        final var b14 = estimator.estimate(position, year);
        assertEquals(b1, b14);

        final var b15 = new NEDMagneticFluxDensity();
        estimator.estimate(position, calendar, b15);
        assertEquals(b1, b15);

        final var b16 = estimator.estimate(position, calendar);
        assertEquals(b1, b16);

        final var b17 = new NEDMagneticFluxDensity();
        estimator.estimate(position, date, b17);
        assertEquals(b1, b17);

        final var b18 = estimator.estimate(position, date);
        assertEquals(b1, b18);
    }

    private static double createYear(final GregorianCalendar calendar) {
        return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar);
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

    private static Date createDate(final long timestamp) {
        return new Date(timestamp);
    }

    private static GregorianCalendar createCalendar(final long timestamp) {
        final var calendar = new GregorianCalendar();
        calendar.setTimeInMillis(timestamp);
        return calendar;
    }
}
