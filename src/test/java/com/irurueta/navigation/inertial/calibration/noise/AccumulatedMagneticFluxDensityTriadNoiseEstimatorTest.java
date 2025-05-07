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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;

import static org.junit.jupiter.api.Assertions.*;

class AccumulatedMagneticFluxDensityTriadNoiseEstimatorTest implements
        AccumulatedMagneticFluxDensityTriadNoiseEstimatorListener {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int N_SAMPLES = 1000;

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

    private int start;
    private int triadAdded;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final var avgX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final var avgY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final var avgZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final var triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final var triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final var norm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final var stdNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final var avgX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final var avgY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final var avgZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final var triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final var triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final var norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final var norm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final var stdNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        // set a new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set a new value
        final var time2 = new Time(500, TimeUnit.MILLISECOND);
        estimator.setTimeInterval(time2);

        // check
        final var time3 = estimator.getTimeIntervalAsTime();
        final var time4 = new Time(0.0, TimeUnit.SECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertTrue(time2.equals(time3, ABSOLUTE_ERROR));
        assertTrue(time2.equals(time4, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testAddTriadAndReset1() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        var triad = new MagneticFluxDensityTriad();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new MagneticFluxDensityTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgX = 0.0;
        var avgY = 0.0;
        var avgZ = 0.0;
        var varX = 0.0;
        var varY = 0.0;
        var varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position, cnb);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(valueX, valueY, valueZ);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final var avgX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final var avgY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final var avgZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgTriad1.getUnit());
        final var avgTriad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgNorm1.getUnit());
        final var avgNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final var stdNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testAddTriadAndReset2() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new MagneticFluxDensityTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgX = 0.0;
        var avgY = 0.0;
        var avgZ = 0.0;
        var varX = 0.0;
        var varY = 0.0;
        var varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position, cnb);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(triad);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final var avgX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final var avgY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final var avgZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgTriad1.getUnit());
        final var avgTriad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgNorm1.getUnit());
        final var avgNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final var stdNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testAddTriadAndReset3() throws LockedException, IOException {
        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final var noiseRandomizer = new GaussianRandomizer(0.0, MAGNETOMETER_NOISE_STD);

        final var position = createPosition(randomizer);
        final var cnb = generateBodyC(randomizer);
        final var timestamp = new Date(createTimestamp(randomizer));

        final var estimator = new AccumulatedMagneticFluxDensityTriadNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, triadAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final var timeInterval = estimator.getTimeInterval();
        final var lastTriad = new MagneticFluxDensityTriad();
        double valueX;
        double valueY;
        double valueZ;
        var avgX = 0.0;
        var avgY = 0.0;
        var avgZ = 0.0;
        var varX = 0.0;
        var varY = 0.0;
        var varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position, cnb);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final var diffX = valueX - avgX;
            final var diffY = valueY - avgY;
            final var diffZ = valueZ - avgZ;

            final var diffX2 = diffX * diffX;
            final var diffY2 = diffY * diffY;
            final var diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, triadAdded);
        assertEquals(0, reset);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final var avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final var avgX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final var avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final var avgY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final var avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final var avgZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final var avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgTriad1.getUnit());
        final var avgTriad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final var avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final var avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgNorm1.getUnit());
        final var avgNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final var stdX = Math.sqrt(varX);
        final var stdY = Math.sqrt(varY);
        final var stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final var stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final var stdX2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final var stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final var stdY2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final var stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final var stdZ2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final var stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final var stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final var stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final var stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final var stdNorm2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final var avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final var avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final var avgStd2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final var psdX = timeInterval * varX;
        final var psdY = timeInterval * varY;
        final var psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final var rootPsdX = Math.sqrt(psdX);
        final var rootPsdY = Math.sqrt(psdY);
        final var rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final var avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final var rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedMagneticFluxDensityTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onTriadAdded(final AccumulatedMagneticFluxDensityTriadNoiseEstimator estimator) {
        triadAdded++;
    }

    @Override
    public void onReset(final AccumulatedMagneticFluxDensityTriadNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        triadAdded = 0;
        reset = 0;
    }

    private void checkLocked(final AccumulatedMagneticFluxDensityTriadNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addTriad(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.addTriad(new MagneticFluxDensityTriad()));
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(LockedException.class, () -> estimator.addTriad(b, b, b));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static MagneticFluxDensityTriad generateTriad(
            final double[] hardIron, final Matrix softIron, final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer, final Date timestamp, final NEDPosition position,
            final CoordinateTransformation cnb) {

        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic.getCoordinatesAsTriad();
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
