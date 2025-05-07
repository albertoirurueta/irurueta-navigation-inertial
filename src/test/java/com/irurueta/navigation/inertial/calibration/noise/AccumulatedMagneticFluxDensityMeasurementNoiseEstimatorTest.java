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

class AccumulatedMagneticFluxDensityMeasurementNoiseEstimatorTest implements 
        AccumulatedMagneticFluxDensityMeasurementNoiseEstimatorListener {

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
    private int measurementAdded;
    private int reset;

    @Test
    void testConstructor1() {
        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testConstructor2() {
        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final var time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        final var time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

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

        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final var timeInterval = estimator.getTimeInterval();
        final var lastMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        double value;
        double avg = 0.0;
        double v = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position, cnb);

            value = triad.getNorm();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final var diff = value - avg;

            final var diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, measurementAdded);
        assertEquals(0, reset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final var std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
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

        final var estimator = new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, measurementAdded);
        assertEquals(0, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final var timeInterval = estimator.getTimeInterval();
        final var lastMeasurement = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.NANOTESLA);
        double value;
        var avg = 0.0;
        var v = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, position, cnb);

            value = triad.getNorm();

            estimator.addMeasurement(triad.getMeasurementNorm());

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final var diff = value - avg;

            final var diff2 = diff * diff;

            v = v * (double) i / (double) j + diff2 / j;
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, measurementAdded);
        assertEquals(0, reset);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final var avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final var avg2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(v, estimator.getVariance(), ABSOLUTE_ERROR);

        final var std = Math.sqrt(v);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final var std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final var std2 = new MagneticFluxDensity(1.0, MagneticFluxDensityUnit.TESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final var psd = timeInterval * v;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final var rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(1, reset);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onMeasurementAdded(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        measurementAdded++;
    }

    @Override
    public void onReset(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        reset++;
    }

    private void reset() {
        start = 0;
        measurementAdded = 0;
        reset = 0;
    }

    private void checkLocked(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setListener(this));
        assertThrows(LockedException.class, () -> estimator.addMeasurement(0.0));
        final var b = new MagneticFluxDensity(0.0, MagneticFluxDensityUnit.TESLA);
        assertThrows(LockedException.class, () -> estimator.addMeasurement(b));
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
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
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
